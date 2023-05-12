#!/usr/bin/env python3
import math
import rospy
import time
import queue
import threading
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_srvs.srv import Empty
from object_tracking.srv import SetTarget, SetTargetResponse, SetTargetRequest
import cv2
import numpy as np
import hiwonder
import sys

# 人脸识别需要的库
sys.path.append("/home/hiwonder/tensorrt_demos")
from utils.mtcnn import TrtMtcnn

mtcnn = TrtMtcnn() # mtcnn 人脸识别网络实例画

image_queue = queue.Queue(maxsize=3) # 图像队列

ROS_NODE_NAME = 'object_tracking'
DEFAULT_X, DEFAULT_Y, DEFAULT_Z = 0, 138 + 8.14, 84 + 128.4
TARGET_PIXEL_X, TARGET_PIXEL_Y = 320, 240



class ObjectTracking:
    def __init__(self):
        self.image_sub = None
        self.heartbeat_timer = None
        self.lock = threading.RLock()
        self.servo_x = 500
        self.servo_y = 500

        # 人脸追踪 pid
        self.face_x_pid = hiwonder.PID(0.09, 0.01, 0.015)
        self.face_z_pid = hiwonder.PID(0.16, 0.0, 0.0)

        # 颜色追踪 pid
        self.color_x_pid = hiwonder.PID(0.07, 0.01, 0.0015)
        self.color_y_pid = hiwonder.PID(0.08, 0.008, 0.001)

        self.tracking_face = None
        self.no_face_count = 0

        self.target_color_range = None
        self.target_color_name = None
        self.last_color_circle = None
        self.lost_target_count = 0

        self.is_running_color = False
        self.is_running_face = False

        self.fps = 0.0
        self.tic = time.time()

    """
    重置变量
    """
    def reset(self):
        self.tracking_face = None
        self.image_sub = None
        self.heartbeat_timer = None
        self.tracking_face_encoding = None
        self.no_face_count = 0
        self.servo_x = 500
        self.servo_y = 500

        self.last_color_circle = None
        self.lost_target_count = 0

        self.is_running_color = False
        self.is_running_face = False

        self.tic = time.time()


# 识别及机械臂操作的接口对象
state = ObjectTracking()
jetmax = hiwonder.JetMax()
sucker = hiwonder.Sucker()


# 初始化， 机械臂回初位
def init():
    state.reset()
    sucker.set_state(False)
    jetmax.go_home(1)

"""
总的图像处理函数
会根据设置状态决定人脸追踪还是颜色追踪
这个函数会在while循环中调用, 而不是直接被ros topic 的回调触发是因为 pycuda 要求相关调用必须在同一个线程中
所以 ros topic 回调只会将接收到的图片放入一个队列而不会直接处理图片。 image_proc 会循环处理该队列
"""
def image_proc():
    # 从队列中获取新的图像
    ros_image = image_queue.get(block=True)
    # 将ros图像转换为opencv图像
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
    if state.is_running_face:
        image = face_tracking(image) # 人脸追踪
    elif state.is_running_color:
        image = color_tracking(image) # 颜色追踪
    else:
        pass
    # 计算帧率
    toc = time.time()
    curr_fps = 1.0 / (state.tic - toc)
    state.fps = curr_fps if state.fps == 0.0 else (state.fps * 0.95 + curr_fps * 0.05)
    state.tic = toc
    # 发布结果图像
    rgb_image = image.tostring()
    ros_image.data = rgb_image
    image_pub.publish(ros_image)


def color_tracking(image):
    org_image = np.copy(image)
    image = cv2.resize(image, (320, 240)) # 缩放一下减少计算量
    image = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # RGB转LAB空间
    image = cv2.GaussianBlur(image, (5, 5), 5)

    with state.lock:
        target_color_range = state.target_color_range
        target_color_name = state.target_color_name

    if target_color_range is not None:
        mask = cv2.inRange(image, tuple(target_color_range['min']), tuple(target_color_range['max']))  # 二值化
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        contour_area = map(lambda c: (c, math.fabs(cv2.contourArea(c))), contours)  # 计算各个轮廓的面积
        contour_area = list(filter(lambda c: c[1] > 200, contour_area))  # 剔除面积过小的轮廓
        circle = None

        if len(contour_area) > 0:
            # 上一帧没有识别到色块
            if state.last_color_circle is None:
                contour, area = max(contour_area, key=lambda c_a: c_a[1])
                circle = cv2.minEnclosingCircle(contour) # 最小外接圆
            else:
            # 上一帧有识别到色块
                (last_x, last_y), last_r = state.last_color_circle
                # 计算新的色块与旧色块的中心距离， 如果太大就认为误识别。放弃这次结果当没有识别到
                circles = map(lambda c: cv2.minEnclosingCircle(c[0]), contour_area) 
                circle_dist = list(map(lambda c: (c, math.sqrt(((c[0][0] - last_x) ** 2) + ((c[0][1] - last_y) ** 2))),
                                       circles))
                circle, dist = min(circle_dist, key=lambda c: c[1])
                if dist < 50:
                    circle = circle

        if circle is not None:
            state.lost_target_count = 0
            (c_x, c_y), c_r = circle
            # 将识别到的坐标重新映射回原始图像大小
            c_x = hiwonder.misc.val_map(c_x, 0, 320, 0, 640)
            c_y = hiwonder.misc.val_map(c_y, 0, 240, 0, 480)
            c_r = hiwonder.misc.val_map(c_r, 0, 320, 0, 640)

            # x 轴 pid 更新
            x = c_x - TARGET_PIXEL_X
            if abs(x) > 30: # 误差在一定范围内就认为到达
                state.color_x_pid.SetPoint = 0
                state.color_x_pid.update(x)
                state.servo_x += state.color_x_pid.output
            else:
                state.color_x_pid.update(0)

            # y 轴 pid 更新
            y = c_y - TARGET_PIXEL_Y
            if abs(y) > 30: # 误差在一定范围内就认为到达
                state.color_y_pid.SetPoint = 0
                state.color_y_pid.update(y)
                state.servo_y -= state.color_y_pid.output
            else:
                state.color_y_pid.update(0)
            # 对舵机范围做限制
            if state.servo_y < 350:
                state.servo_y = 350
            if state.servo_y > 650:
                state.servo_y = 650

            # 执行新的目标位置
            jetmax.set_servo(1, int(state.servo_x), duration=0.02)
            jetmax.set_servo(2, int(state.servo_y), duration=0.02)
            color_name = target_color_name.upper()
            # 画面显示识别结果
            org_image = cv2.circle(org_image, (int(c_x), int(c_y)), int(c_r), hiwonder.COLORS[color_name], 3)
            state.last_color_circle = circle
        else:
            # 超过一定次数没有识别到就清理标志重新识别
            state.lost_target_count += 1
            if state.lost_target_count > 15:
                state.lost_target_count = 0
                state.last_color_circle = None
    return org_image

"""
画面中显示人脸
并返回人脸识别到的相关坐标在原图上的对应坐标
"""
def show_faces(img, boxes, landmarks, color):
    new_boxes = []
    new_landmarks = []
    for bb, ll in zip(boxes, landmarks):
        # 将坐标映射回原图坐标
        x1 = int(hiwonder.misc.val_map(bb[0], 0, 400, 0, 640))
        y1 = int(hiwonder.misc.val_map(bb[1], 0, 300, 0, 480))
        x2 = int(hiwonder.misc.val_map(bb[2], 0, 400, 0, 640))
        y2 = int(hiwonder.misc.val_map(bb[3], 0, 300, 0, 480))
        # 画出 外框
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        new_boxes.append([x1, y1, x2, y2])
        landmarks_curr_face = []
        if len(landmarks):
            # 画出关键点
            for j in range(5):
                # 将坐标映射回原图坐标
                lx = int(hiwonder.misc.val_map(ll[j], 0, 400, 0, 640))
                ly = int(hiwonder.misc.val_map(ll[j + 5], 0, 300, 0, 480))
                cv2.circle(img, (lx, ly), 2, color, 2)
                landmarks_curr_face.append([lx, ly])
        new_landmarks.append(landmarks_curr_face)
    return img, new_boxes, new_landmarks


def face_tracking(image):
    global mtcnn
    ta = time.time()
    org_img = np.copy(image)
    image = cv2.resize(image, (400, 300)) # 缩放一下减少计算量

    boxes, landmarks = mtcnn.detect(image, minsize=40) # 调用 mtcnn 进行人脸识别
    org_img, boxes, landmarks = show_faces(org_img, boxes, landmarks, (0, 255, 0)) # 显示人脸

    # 上一帧没有识别到人脸
    if state.tracking_face is None:
        if len(boxes) > 0:  # 如果识别到人脸就找出距离画面中心最近的人脸来追踪
            box = min(boxes, key=lambda b: math.sqrt(((b[2] + b[0]) / 2 - 320) ** 2 + ((b[3] + b[1]) / 2 - 240) ** 2))
            x1, y1, x2, y2 = box
            # 用红色单独画出要追踪的人脸
            org_img = cv2.rectangle(org_img, (x1, y1), (x2, y2), (255, 0, 0), 3)
            center_x, center_y = int((x1 + x2) / 2), int((y1 + y2) / 2)
            org_img = cv2.circle(org_img, (center_x, center_y), 2, (0, 255, 0), 4)
            # 记录要追踪的人脸的坐标
            state.tracking_face = center_x, center_y
            state.no_face_count = time.time() + 2
            # 清理pid控制器
            state.face_x_pid.clear()
            state.face_z_pid.clear()
    else: # 上一帧识别到了人脸
        # 计算所有识别到的人脸的中心坐标
        centers = [(int((box[2] + box[0]) / 2), int((box[3] + box[1]) / 2), box) for box in boxes]
        get_face = False
        # 找出距离上一帧识别到的人脸最近的人脸 
        if len(centers) > 0:
            center_x, center_y = state.tracking_face
            org_img = cv2.circle(org_img, (center_x, center_y), 2, (0, 0, 255), 4)
            min_dist_center = min(centers, key=lambda c: math.sqrt((c[0] - center_x) ** 2 + (c[1] - center_y) ** 2))
            new_center_x, new_center_y, box = min_dist_center
            x1, y1, x2, y2 = box
            dist = math.sqrt((new_center_x - center_x) ** 2 + (new_center_y - center_y) ** 2)
            # 像素距离小于一定值才认为是同一张人脸， 超过就当没识别到
            if dist < 150:
                org_img = cv2.rectangle(org_img, (x1, y1), (x2, y2), (255, 0, 0), 3)
                org_img = cv2.circle(org_img, (new_center_x, new_center_y), 2, (255, 0, 0), 4)
                get_face = True
                state.tracking_face = int(new_center_x), int(new_center_y)
                state.no_face_count = time.time() + 2
        # 判断是否超过一定时间没有识别要追踪的人脸
        if state.no_face_count < time.time():
            state.tracking_face = None

        # 本帧中识别到了要追踪的人脸
        if get_face:
            center_x, center_y = state.tracking_face
            x = center_x - 320
            # 更新 x 轴 pid控制器
            if abs(x) > 30:
                state.face_x_pid.SetPoint = 0
                state.face_x_pid.update(x)
                state.servo_x += state.face_x_pid.output
                jetmax.set_servo(1, int(state.servo_x), 0.08)
            else:
                state.face_x_pid.update(0)

            # 更新 z 轴 pid 控制器
            z = center_y - 240
            if abs(z) > 30:
                state.face_z_pid.SetPoint = 0
                state.face_z_pid.update(z)
                z = jetmax.position[2] + state.face_z_pid.output
                jetmax.set_position([jetmax.position[0], jetmax.position[1], z], 0.08)
            else:
                state.face_z_pid.update(0)
    return org_img


"""
相机画面topic的回调
只会将接收到的画面推入队列
"""
def image_callback(ros_image):
    try:
        image_queue.put_nowait(ros_image)
    except queue.Full:
        pass


"""
启动服务
程序进入就绪状态，准备运行，但不做识别
"""
def enter_func(msg):
    rospy.loginfo("enter object tracking")
    exit_func(msg)  # 先退出一次, 简化过程
    rospy.sleep(0.1)
    init()  # 初始化位置及状态
    state.target_color_range = None
    state.target_color_name = None
    state.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, image_callback)  # 订阅摄像头画面
    rospy.ServiceProxy('/usb_cam/start_capture', Empty)()
    return [True, '']


def reset():
    exit_func(TriggerRequest())  # 先退出一次, 简化过程
    rospy.sleep(0.1)
    init()  # 初始化位置及状态
    state.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, image_callback)  # 订阅摄像头画面
    rospy.ServiceProxy('/usb_cam/start_capture', Empty)()

"""
退出服务
程序取消对相机topic的订阅
"""
def exit_func(msg):
    rospy.loginfo("exit object tracking")
    state.is_running_color = False
    state.is_running_face = False
    try:
        state.heartbeat_timer.cancel()
    except:
        pass
    try:  # 注销图像节点的订阅, 这样回调就不会被调用了
        rospy.loginfo('unregister image')
        state.image_sub.unregister()
    except:
        pass
    finally:
        state.image_sub = None
    return TriggerResponse(success=True)


"""
开始颜色追踪
"""
def set_running_color_cb(msg: SetBoolRequest):
    reset()
    if msg.data:
        rospy.loginfo("start running color tracking")
        state.is_running_face = False
        state.is_running_color = True
    else:
        rospy.loginfo("stop running color tracking")
        state.is_running_color = False
    return SetBoolResponse(success=True)


"""
开始人脸追踪
"""
def set_running_face_cb(msg: SetBoolRequest):
    reset()
    if msg.data:
        rospy.loginfo("start running face tracking")
        state.is_running_color = False
        state.is_running_face = True
    else:
        rospy.loginfo("stop running face tracking")
        state.is_running_face = False
    return [True, '']


def heartbeat_timeout_cb():
    rospy.loginfo("heartbeat timeout. exiting...")
    rospy.ServiceProxy('/%s/exit' % ROS_NODE_NAME, Trigger)()


def heartbeat_srv_cb(msg: SetBoolRequest):
    """
    心跳回调.会设置一个定时器，当定时到达后会调用退出服务退出玩法处理
    :params msg: 服务调用参数， std_srv SetBool
    """
    rospy.logdebug("Heartbeat")
    try:
        state.heartbeat_timer.cancel()
    except:
        pass
    if msg.data:
        state.heartbeat_timer = threading.Timer(20, heartbeat_timeout_cb)
        state.heartbeat_timer.start()
    return SetBoolResponse(success=msg.data)


"""
设置要追踪的目标颜色
"""
def set_target_cb(msg: SetTargetRequest):
    # 获取颜色阈值列表
    color_ranges = rospy.get_param('/lab_config_manager/color_range_list', {})
    rospy.logdebug(color_ranges)
    with state.lock:
        if msg.color_name in color_ranges:
            # 从阈值列表中找到目标阈值， 并设置目标
            state.target_color_name = msg.color_name
            state.target_color_range = color_ranges[msg.color_name]
            return [True, '']
        else:
            state.target_color_name = None
            state.target_color_range = None
            return [False, '']


if __name__ == '__main__':
    # 初始化节点
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    # 初始化程序资源
    init()

    # topic订阅/发布注册，服务注册
    image_pub = rospy.Publisher('/%s/image_result' % ROS_NODE_NAME, Image, queue_size=1)  # register result image pub
    enter_srv = rospy.Service('/%s/enter' % ROS_NODE_NAME, Trigger, enter_func)
    exit_srv = rospy.Service('/%s/exit' % ROS_NODE_NAME, Trigger, exit_func)
    color_running_srv = rospy.Service('/%s/set_running_color' % ROS_NODE_NAME, SetBool, set_running_color_cb)
    face_running_srv = rospy.Service('/%s/set_running_face' % ROS_NODE_NAME, SetBool, set_running_face_cb)
    set_target_srv = rospy.Service("/%s/set_target" % ROS_NODE_NAME, SetTarget, set_target_cb)
    heartbeat_srv = rospy.Service('/%s/heartbeat' % ROS_NODE_NAME, SetBool, heartbeat_srv_cb)
    hiwonder.buzzer.on()
    rospy.sleep(0.2)
    hiwonder.buzzer.off()
    while True:
        try:
            image_proc() # 循环处理图像
            if rospy.is_shutdown():
                break
        except KeyboardInterrupt:
            break
