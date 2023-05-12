#!/usr/bin/env python3
import os
import sys
import cv2
import math
import time
import queue
import random
import threading
import numpy as np
import rospy
import hiwonder
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from std_srvs.srv import Trigger, TriggerResponse
from jetmax_control.msg import SetServo
from yolov5_tensorrt import Yolov5TensorRT

ROS_NODE_NAME = "alphabetically"
IMAGE_SIZE = 640, 480 # 输入图片尺寸

CHARACTERS_ENGINE_PATH = os.path.join(sys.path[0], 'characters_v5_160.trt') # 字母模型路径
CHARACTER_LABELS = tuple([i for i in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ']) # 字母分类名称
CHARACTER_NUM = 26

NUMBERS_ENGINE_PATH = os.path.join(sys.path[0], 'numbers_v5_160.trt') # 数字模型路径
NUMBERS_LABELS = tuple([i for i in '0123456789+-*/=']) # 数字分类数值的名称
NUMBERS_NUM = 15 # 分类总个数

TRT_INPUT_SIZE = 160 # trt输入尺寸
# 随机颜色， 用于结果框出
COLORS = tuple([tuple([random.randint(10, 255) for j in range(3)]) for i in range(CHARACTER_NUM + NUMBERS_NUM)])

GOAL_POSITIONS = (
    (-230, -85, 55, 17), (-230, -40, 55, 7), (-230, 5, 55, -3), (-230, 50, 55, -13),
    (-185, -85, 55, 20), (-185, -40, 55, 10), (-185, 5, 55, -3), (-185, 50, 55, -13),
    (-140, -85, 55, 30), (-140, -40, 55, 15), (-140, 5, 55, -5), (-140, 50, 55, -18),
)


def camera_to_world(cam_mtx, r, t, img_points):
    inv_k = np.asmatrix(cam_mtx).I
    r_mat = np.zeros((3, 3), dtype=np.float64)
    cv2.Rodrigues(r, r_mat)
    # invR * T
    inv_r = np.asmatrix(r_mat).I  # 3*3
    transPlaneToCam = np.dot(inv_r, np.asmatrix(t))  # 3*3 dot 3*1 = 3*1
    world_pt = []
    coords = np.zeros((3, 1), dtype=np.float64)
    for img_pt in img_points:
        coords[0][0] = img_pt[0][0]
        coords[1][0] = img_pt[0][1]
        coords[2][0] = 1.0
        worldPtCam = np.dot(inv_k, coords)  # 3*3 dot 3*1 = 3*1
        # [x,y,1] * invR
        worldPtPlane = np.dot(inv_r, worldPtCam)  # 3*3 dot 3*1 = 3*1
        # zc
        scale = transPlaneToCam[2][0] / worldPtPlane[2][0]
        # zc * [x,y,1] * invR
        scale_worldPtPlane = np.multiply(scale, worldPtPlane)
        # [X,Y,Z]=zc*[x,y,1]*invR - invR*T
        worldPtPlaneReproject = np.asmatrix(scale_worldPtPlane) - np.asmatrix(transPlaneToCam)  # 3*1 dot 1*3 = 3*3
        pt = np.zeros((3, 1), dtype=np.float64)
        pt[0][0] = worldPtPlaneReproject[0][0]
        pt[1][0] = worldPtPlaneReproject[1][0]
        pt[2][0] = 0
        world_pt.append(pt.T.tolist())
    return world_pt


class Alphabetically:
    def __init__(self):
        self.running_mode = 0
        self.moving_box = None
        self.image_sub = None
        self.heartbeat_timer = None
        self.runner = None
        self.moving_count = 0
        self.count = 0
        self.lock = threading.RLock()
        self.fps_t0 = time.time()
        self.fps = 0
        self.camera_params = None
        self.enable_moving = False
        self.K = None
        self.R = None
        self.T = None

    """
    重置需要的变量
    """
    def reset(self):
        self.running_mode = 0
        self.enable_moving = False
        self.moving_box = None
        self.moving_count = 0
        self.image_sub = None
        self.heartbeat_timer = None
        self.runner = None
        self.count = 0
        self.fps_t0 = time.time()
        self.fps = 0

    """
    读取相机内参
    """
    def load_camera_params(self):
        self.camera_params = rospy.get_param('/camera_cal/card_params', self.camera_params)
        if self.camera_params is not None:
            self.K = np.array(self.camera_params['K'], dtype=np.float64).reshape(3, 3)
            self.R = np.array(self.camera_params['R'], dtype=np.float64).reshape(3, 1)
            self.T = np.array(self.camera_params['T'], dtype=np.float64).reshape(3, 1)


"""
移动卡片
"""
def moving():
    try:
        c_x, c_y, cls_id, cls_conf = state.moving_box # 获取要移动的卡片在图上的位置
        cur_x, cur_y, cur_z = jetmax.position # 吸嘴的当前坐标

        # 计算卡片在现实世界的中吸嘴中心(外参标定的时候用吸嘴标定的)的坐标
        x, y, _ = camera_to_world(state.K, state.R, state.T, np.array([c_x, c_y]).reshape((1, 1, 2)))[0][0]
        
        t = math.sqrt(x * x + y * y + 120 * 120) / 120 # 计算卡片位置的距离, 通过距离/速度=时间, 计算用多少时间到达卡片位置
        new_x, new_y = cur_x + x, cur_y + y # 计算卡片位置相对于机械臂基座的坐标
        nn_new_x = new_x + 15
        arm_angle = math.atan(new_y / new_x) * 180 / math.pi # 计算机械臂到达新位置后相对与中轴的偏转角度, 后边我们放置卡片的时候给小舵机旋转给偏回来
        if arm_angle > 0:
            arm_angle = (90 - arm_angle)
        elif arm_angle < 0:
            arm_angle = (-90 - arm_angle)
        else:
            pass
        # 机械臂分步运行到卡片位置
        jetmax.set_position((nn_new_x, new_y, 70), t)
        rospy.sleep(t)
        jetmax.set_position((new_x, new_y, 70), 0.3)
        rospy.sleep(t + 0.6)

        # 下降，吸取
        sucker.set_state(True)
        jetmax.set_position((new_x, new_y, 50), 0.8)
        rospy.sleep(0.85)

        # 根据顺序查表获取要放到哪里, 并抬起机械臂
        x, y, z, angle = GOAL_POSITIONS[state.moving_count]
        cur_x, cur_y, cur_z = jetmax.position
        jetmax.set_position((cur_x, cur_y, 100), 0.8)
        rospy.sleep(0.8)

        # 控制小舵机旋转卡片，使卡片角度能够跟吸取之前的放置角度一样
        hiwonder.pwm_servo1.set_position(90 + angle + arm_angle, 0.1)
        cur_x, cur_y, cur_z = jetmax.position
        # 计算当前位置到目标位置的距离以控制速度
        t = math.sqrt((cur_x - x) ** 2 + (cur_y - y) ** 2) / 150
        # 控制机械臂到达目标上方
        jetmax.set_position((x, y, z + 30), t)
        rospy.sleep(t)

        # 控制机械臂到达目标位置
        jetmax.set_position((x, y, z), 0.8)
        rospy.sleep(0.8)

        #释放
        sucker.release(3)
        #抬起
        jetmax.set_position((x, y, z + 30), 0.8)
        rospy.sleep(0.1)
        #小舵机恢复
        hiwonder.pwm_servo1.set_position(90, 0.4)
        rospy.sleep(0.8)

    finally:
        sucker.release(3)
        cur_x, cur_y, cur_z = jetmax.position
        # 计算当期位置和默认位置的距离以控制速度
        t = math.sqrt((cur_x - jetmax.ORIGIN[0]) ** 2 + (cur_y - jetmax.ORIGIN[1]) ** 2) / 120
        # 回到默认位置
        jetmax.go_home(t)
        # 小舵机恢复, 多次恢复是因为，上面的恢复可能因为异常而未被执行
        hiwonder.pwm_servo1.set_position(90, 0.2)
        rospy.sleep(t + 0.2)
        with state.lock: # 清理运行标识，让程序继续下次操作
            state.moving_box = None
            state.moving_count += 1 # 累计摆放的个数，以记录放到哪里去
            if state.moving_count >= len(GOAL_POSITIONS):
                state.moving_count = 0
            state.runner = None
        print("FINISHED")


"""
字母图像处理
"""
def image_proc_chars(img_in):
    if state.runner is not None: # 如果有搬运动作在执行就不进行识别
        return img_in
    result_image = img_in
    outputs = yolov5_chars.detect(np.copy(img_in)) # 调用yolov5 进行识别
    boxes, confs, classes = yolov5_chars.post_process(img_in, outputs, 0.65) # 对 yolov5 的网络输出进行后处理， 获取最终结果
    cards = []
    width, height = IMAGE_SIZE
    # 遍历识别出的结果
    for box, cls_id, cls_conf in zip(boxes, classes, confs):
        x1 = box[0] / TRT_INPUT_SIZE * width
        y1 = box[1] / TRT_INPUT_SIZE * height
        x2 = box[2] / TRT_INPUT_SIZE * width
        y2 = box[3] / TRT_INPUT_SIZE * height
        cards.append((x1, y1, x2, y2, cls_id, cls_conf))
        # 画面中框出识别的的卡片并加上显示
        cv2.putText(img_in, CHARACTER_LABELS[cls_id] + " " + str(float(cls_conf))[:4],
                    (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORS[cls_id], 2)
        cv2.rectangle(result_image, (int(x1), int(y1)), (int(x2), int(y2)), COLORS[cls_id], 3)

    if len(cards) == 0: # 没有识别到
        state.count = 0
        state.moving_box = None
    else: # 识别到
        if state.moving_box is None:  # 上一帧没有识别到， 要进行多次识别以防误触发
            moving_box = min(cards, key=lambda x: x[4]) # 所有识别到的卡片中 id 最小的
            x1, y1, x2, y2, cls_id, cls_conf = moving_box
            c_x, c_y = (x1 + x2) / 2, (y1 + y2) / 2 # 框的中心
            state.moving_box = c_x, c_y, cls_id, cls_conf # 记录这个卡片的位置等参数
            cv2.circle(result_image, (int(c_x), int(c_y)), 1, (255, 0, 0), 30) # 画出中心
            state.count = 0
        else: # 上一帧有识别到
            l_c_x, l_c_y, l_cls_id, _ = state.moving_box
            # 遍历所有新识别到的卡片
            cards = [((x1 + x2) / 2,
                      (y1 + y2) / 2,
                      cls_id, cls_conf) for x1, y1, x2, y2, cls_id, cls_conf in cards]
            # 找出与上一帧记录的目标卡片距离最近的卡片
            distances = [math.sqrt((l_c_x - c_x) ** 2 + (l_c_y - c_y) ** 2) for c_x, c_y, _, _ in cards]
            new_moving_box = min(zip(distances, cards), key=lambda x: x[0])
            _, (c_x, c_y, cls_id, cls_conf) = new_moving_box
            # 画出两次的中心
            cv2.circle(result_image, (int(l_c_x), int(l_c_y)), 1, (0, 255, 0), 30)
            cv2.circle(result_image, (int(c_x), int(c_y)), 1, (255, 0, 0), 30)
            if cls_id == l_cls_id and state.enable_moving: # 如果两次id相同就是正确识别
                state.moving_box = c_x, c_y, cls_id, cls_conf
                state.count += 1
                if state.count > 20: # 连续20帧识别到就开始搬运卡片
                    state.runner = threading.Thread(target=moving, daemon=True)
                    state.runner.start()
            else:
                state.moving_box = None
    return result_image

"""
数字卡片识别
整体过程跟字母卡片相同， 只是用的模型不同
"""
def image_proc_nums(img_in):
    if state.runner is not None:
        return img_in
    result_image = img_in
    outputs = yolov5_nums.detect(np.copy(img_in))
    boxes, confs, classes = yolov5_nums.post_process(img_in, outputs, 0.65)
    cards = []
    width, height = IMAGE_SIZE

    for box, cls_id, cls_conf in zip(boxes, classes, confs):
        x1 = box[0] / TRT_INPUT_SIZE * width
        y1 = box[1] / TRT_INPUT_SIZE * height
        x2 = box[2] / TRT_INPUT_SIZE * width
        y2 = box[3] / TRT_INPUT_SIZE * height
        cards.append((x1, y1, x2, y2, cls_id, cls_conf))
        result_image = cv2.putText(img_in, NUMBERS_LABELS[cls_id] + " " + str(float(cls_conf))[:4],
                                   (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                   COLORS[CHARACTER_NUM + cls_id], 2)
        result_image = cv2.rectangle(result_image, (int(x1), int(y1)), (int(x2), int(y2)),
                                     COLORS[CHARACTER_NUM + cls_id], 3)

    if len(cards) == 0:
        state.count = 0
        state.moving_box = None
    else:
        if state.moving_box is None:
            moving_box = min(cards, key=lambda x: x[4])
            x1, y1, x2, y2, cls_id, cls_conf = moving_box
            c_x, c_y = (x1 + x2) / 2, (y1 + y2) / 2
            state.moving_box = c_x, c_y, cls_id, cls_conf
            result_image = cv2.circle(result_image, (int(c_x), int(c_y)), 1, (255, 0, 0), 30)
            state.count = 0
        else:
            l_c_x, l_c_y, l_cls_id, _ = state.moving_box
            cards = [((x1 + x2) / 2,
                      (y1 + y2) / 2,
                      cls_id, cls_conf) for x1, y1, x2, y2, cls_id, cls_conf in cards]
            distances = [math.sqrt((l_c_x - c_x) ** 2 + (l_c_y - c_y) ** 2) for c_x, c_y, _, _ in cards]
            new_moving_box = min(zip(distances, cards), key=lambda x: x[0])
            _, (c_x, c_y, cls_id, cls_conf) = new_moving_box
            result_image = cv2.circle(result_image, (int(l_c_x), int(l_c_y)), 1, (0, 255, 0), 30)
            result_image = cv2.circle(result_image, (int(c_x), int(c_y)), 1, (255, 0, 0), 30)
            if cls_id == l_cls_id and state.enable_moving:
                state.moving_box = c_x, c_y, cls_id, cls_conf
                state.count += 1
                if state.count > 20:
                    state.runner = threading.Thread(target=moving, daemon=True)
                    state.runner.start()
            else:
                state.moving_box = None
    return result_image


def show_fps(img, fps):
    """显示fps"""
    font = cv2.FONT_HERSHEY_PLAIN
    line = cv2.LINE_AA
    fps_text = 'FPS: {:.2f}'.format(fps)
    cv2.putText(img, fps_text, (11, 20), font, 1.0, (32, 32, 32), 4, line)
    cv2.putText(img, fps_text, (10, 20), font, 1.0, (240, 240, 240), 1, line)
    return img


"""
总的图像处理函数
会根据设置状态决定调用数字卡片识别还是字母卡片识别或是不识别
这个函数会在while循环中调用, 而不是直接被ros topic 的回调触发是因为 pycuda 要求相关调用必须在同一个线程中
所以 ros topic 回调只会将接收到的图片放入一个队列而不会直接处理图片。 image_proc 会循环处理该队列
"""
def image_proc():
    ros_image = image_queue.get(block=True)
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
    with state.lock:
        if state.running_mode == 1:
            result_img = image_proc_chars(image)
        elif state.running_mode == 2:
            result_img = image_proc_nums(image)
        else:
            result_img = image
    # fps cal
    fps_t1 = time.time()
    fps_cur = (1.0 / (fps_t1 - state.fps_t0))
    state.fps = fps_cur if state.fps == 0.0 else (state.fps * 0.8 + fps_cur * 0.2)
    state.fps_t0 = fps_t1
    # show_fps(result_img, state.fps)
    #
    rgb_image = result_img.tostring()
    ros_image.data = rgb_image
    image_pub.publish(ros_image)


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
    rospy.loginfo("enter")
    exit_func(msg)
    jetmax.go_home()
    state.reset()
    state.load_camera_params()
    state.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, image_callback)
    return TriggerResponse(success=True)


"""
退出服务
程序取消对相机topic的订阅
"""
def exit_func(msg):
    rospy.loginfo("exit")
    try:
        state.heartbeat_timer.cancel()
    except:
        pass
    with state.lock:
        state.running_mode = 0
        # 取消订阅
        if isinstance(state.image_sub, rospy.Subscriber):
            rospy.loginfo('unregister image')
            state.image_sub.unregister()
            state.image_sub = None
    # 等待正在运行的搬运动作完成
    if isinstance(state.runner, threading.Thread):
        state.runner.join()
    # 调用服务让机械臂回到初始位置
    rospy.ServiceProxy('/jetmax/go_home', Empty)()
    rospy.Publisher('/jetmax/end_effector/sucker/command', Bool, queue_size=1).publish(data=False)
    rospy.Publisher('/jetmax/end_effector/servo1/command', SetServo, queue_size=1).publish(data=90, duration=0.5)
    return TriggerResponse(success=True)


def heartbeat_timeout_cb():
    rospy.loginfo("heartbeat timeout. exiting...")
    rospy.ServiceProxy('/%s/exit' % ROS_NODE_NAME, Trigger)


def heartbeat_srv_cb(msg: SetBoolRequest):
    try:
        state.heartbeat_timer.cancel()
    except:
        pass
    rospy.logdebug("Heartbeat")
    if msg.data:
        state.heartbeat_timer = threading.Timer(5, heartbeat_timeout_cb)
        state.heartbeat_timer.start()
    return SetBoolResponse(success=msg.data)


"""
开始字母卡片识别
"""
def set_char_running_cb(msg):
    with state.lock:
        if msg.data:
            # 如果当前模式不是字母识别模式(1), 就设为字母识别模式
            if state.running_mode != 1:
                state.running_mode = 1
                state.moving_count = 0
                state.enable_moving = False
            # 如果已经是字母识别模式就切换识别开关
            else:
                if state.enable_moving:
                    state.enable_moving = False
                else:
                    state.enable_moving = True
        else:
            if state.running_mode == 1:
                state.running_mode = 0
                state.enable_moving = False
    return [True, '']


"""
开始识别数字卡片
"""
def set_num_running_cb(msg):
    with state.lock:
        if msg.data:
            # 如果 当前运行模式不是识别数字就改为识别数字
            if state.running_mode != 2:
                state.running_mode = 2
                state.moving_count = 0
                state.enable_moving = False
            # 如果是识别数字， 就控制开关识别
            else:
                if state.enable_moving:
                    state.enable_moving = False
                else:
                    state.enable_moving = True
        else:
            if state.running_mode == 2:
                state.running_mode = 0
                state.enable_moving = False
    return [True, '']



if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG) # 初始化节点
    state = Alphabetically() # 初始化相关参数
    state.load_camera_params() # 读取相机内参
    if state.camera_params is None:
        rospy.logerr("Can not load camera parameters")
        sys.exit(-1)
    # 建立字母、数字识别器
    yolov5_chars = Yolov5TensorRT(CHARACTERS_ENGINE_PATH, TRT_INPUT_SIZE, CHARACTER_NUM)
    yolov5_nums = Yolov5TensorRT(NUMBERS_ENGINE_PATH, TRT_INPUT_SIZE, NUMBERS_NUM)
    # 图像队列
    image_queue = queue.Queue(maxsize=1)
    # 机器人的控制接口
    jetmax = hiwonder.JetMax()
    sucker = hiwonder.Sucker()

    # 相关topic的订阅和发布注册
    image_pub = rospy.Publisher('/%s/image_result' % ROS_NODE_NAME, Image, queue_size=1)  # register result image pub
    enter_srv = rospy.Service('/%s/enter' % ROS_NODE_NAME, Trigger, enter_func)
    exit_srv = rospy.Service('/%s/exit' % ROS_NODE_NAME, Trigger, exit_func)
    char_running_srv = rospy.Service('/%s/set_running_char' % ROS_NODE_NAME, SetBool, set_char_running_cb)
    num_running_srv = rospy.Service('/%s/set_running_num' % ROS_NODE_NAME, SetBool, set_num_running_cb)
    heartbeat_srv = rospy.Service('/%s/heartbeat' % ROS_NODE_NAME, SetBool, heartbeat_srv_cb)

    while True:
        try:
            image_proc() # 循环处理图片
            if rospy.is_shutdown():
                break
        except KeyboardInterrupt:
            rospy.signal_shutdown("custom shutdown")
            break
