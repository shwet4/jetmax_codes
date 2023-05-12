#!/usr/bin/env python3
import os
import sys
import cv2
import threading
import rospy
import numpy as np
import yaml
import hiwonder
import apriltag
from sensor_msgs.msg import Image
from jetmax_control.msg import SetJetMax
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_srvs.srv import Empty
from sensor_msgs.msg import CameraInfo

ROS_NODE_NAME = "camera_cal"
at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11')) # apriltag检测器

# apriltag 中心在原点的情况下， 中心及四个角的坐标
marker_corners = np.asarray([[0, 0, 40], # 40为木块高度
                             [16.65, -16.65, 40],  # TAG_SIZE = 33.30mm
                             [-16.65, -16.65, 40],
                             [-16.65, 16.65, 40],
                             [16.65, 16.65, 40]],
                            dtype=np.float64)

# 对不同高度的物体(卡片和木块)我们直接用不同高度算出不同的的参数
marker_corners_block = np.copy(marker_corners)
marker_corners_block[:, 2] = marker_corners_block[:, 2] - 40
jetmax = hiwonder.JetMax()

class State:
    def __init__(self):
        self.lock = threading.RLock()
        # 读取摄像头内参
        with open('/home/hiwonder/.ros/camera_info/head_camera.yaml') as f:
            camera_params = yaml.load(f.read())
            K_ = np.asarray([camera_params['camera_matrix']['data']]).reshape((3, 3))
            self.D = np.asarray(camera_params['distortion_coefficients']['data'])
            self.K = cv2.getOptimalNewCameraMatrix(K_, self.D, (640, 480), 0, (640, 480))[0]
        self.heartbeat_timer = None
        self.image_sub = None
        self.is_running = False
        self.enter = False
        self.R = None
        self.T = None
        self.R_40 = None
        self.T_40 = None

    """
    重置变量
    """
    def reset(self):
        self.is_running = False
        self.enter = False
        self.R = None
        self.T = None
        self.R_40 = None
        self.T_40 = None


"""
像素坐标转换为世界坐标
"""
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


def image_proc(img):
    frame_gray = cv2.cvtColor(np.copy(img), cv2.COLOR_RGB2GRAY)
    tags = at_detector.detect(frame_gray) # 识别apriltag
    for tag in tags:
        center = np.array(tag.center).astype(int) # 中心点坐标
        if tag.tag_id == 1: # 只有 id 为 1 的tag做标定
            corners = tag.corners.reshape(1, -1, 2).astype(np.float32)
            pts = np.insert(corners[0], 0, values=tag.center, axis=0)
            with state.lock:
                # 计算卡片情况下相机二维坐标与世界三维坐标的转换关系
                rtl, new_r, new_t = cv2.solvePnP(marker_corners, pts, state.K, None)
                if rtl:
                    # 有简单的滤波避免跳动带来的影响
                    state.R = new_r if state.R is None else state.R * 0.95 + new_r * 0.05
                    state.T = new_t if state.T is None else state.T * 0.95 + new_t * 0.05

                # 计算木块情况下相机二维坐标与世界三维坐标的转换关系
                rtl, new_r, new_t = cv2.solvePnP(marker_corners_block, pts, state.K, None)
                if rtl:
                    # 有简单的滤波避免跳动带来的影响
                    state.R_40 = new_r if state.R_40 is None else state.R_40 * 0.95 + new_r * 0.05
                    state.T_40 = new_t if state.T_40 is None else state.T_40 * 0.95 + new_t * 0.05

                if state.R is not None:# 有成功计算到转换关系的情况下
                    # 用计算得到的转换关系和实际物理坐标反算出标签四个角在画面上的理论像素位置， 
                    # 如果理论像素位置和识别到的标签实际像素位置重合就是计算得到的转换关系符合实际情况，就是准确的
                    img_pts, jac = cv2.projectPoints(marker_corners, state.R, state.T, state.K, None)
                else:
                    img_pts = []
            corners = corners.astype(int)
            # 彩色画出标签的四个角和中心
            cv2.circle(img, tuple(corners[0][0]), 5, (255, 0, 0), 12)
            cv2.circle(img, tuple(corners[0][1]), 5, (0, 255, 0), 12)
            cv2.circle(img, tuple(corners[0][2]), 5, (0, 0, 255), 12)
            cv2.circle(img, tuple(corners[0][3]), 5, (0, 255, 255), 12)
            cv2.circle(img, tuple(center), 5, (255, 255, 0), 12)
            cv2.putText(img, "id:%d" % tag.tag_id,
                        (center[0], center[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            # 黑色画出反算出的理论像素位置
            # 黑色彩色重合就是有效了
            for i, c in enumerate(img_pts):
                cv2.circle(img, tuple(c.astype(int)[0]), 3, (0, 0, 0), 4)
        else:
            with state.lock:
                # 对于其他id不为1的标签，我们用得到的转换关系计算他们的世界坐标，并显示在画面上
                # 这样也可以测量实际位置和显示的位置是否一致判断标定的准确性。
                if state.R_40 is not None:
                    w = camera_to_world(state.K, state.R_40, state.T_40,
                                        center.reshape((1, 1, 2)))[0][0]
                else:
                    w = 0, 0, 0
            cv2.putText(img, "id:{} x:{:.2f} y:{:.2f}".format(tag.tag_id, w[0], w[1]),
                        (center[0] - 20, center[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    return img


def image_callback(ros_image):
    if state.enter is True:
        # 将ros格式图转换为opencv格式
        image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        frame_result = image.copy()
        with state.lock:
            # 进行识别处理
            frame_result = image_proc(frame_result)
        # 发布结果图像
        rgb_image = frame_result.tostring()
        ros_image.data = rgb_image
        image_pub.publish(ros_image)

"""
进入服务
"""
def enter_func(msg):
    rospy.loginfo("enter object tracking")
    exit_func(msg)
    state.reset()
    state.enter = True
    return TriggerResponse(success=True)

"""
退出服务
"""
def exit_func(msg):
    rospy.loginfo("exit object tracking")
    state.is_running = False
    state.enter = False
    try:
        state.heartbeat_timer.cancel()
    except:
        pass
    #if isinstance(state.image_sub, rospy.Subscriber):
    #    rospy.loginfo('unregister image')
    #    state.image_sub.unregister()
    #    state.image_sub = None
    rospy.ServiceProxy('/jetmax/go_home', Empty)()
    #up_cb(msg)
    return TriggerResponse(success=True)

"""
开始/停止标定
"""
def set_running(msg: SetBoolRequest):
    rospy.loginfo('set running' + str(msg))
    with state.lock:
        state.is_running = msg.data
    return [True, '']


def heartbeat_timeout_cb():
    rospy.loginfo('heartbeat timeout. exiting')
    rospy.ServiceProxy('/%s/exit' % ROS_NODE_NAME, Trigger)()


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
将当前的标定数据保存到文件中
"""
def save_cb(msg):
    rospy.loginfo("save")
    with state.lock:
        card_params = {
            "K": state.K.tolist(),
            "R": state.R.tolist(),
            "T": state.T.tolist()
        }
        block_params = {
            "K": state.K.tolist(),
            "R": state.R_40.tolist(),
            "T": state.T_40.tolist()
        }
        s = yaml.dump({
            'card_params': card_params,
            'block_params': block_params,
        }, default_flow_style=True)
        rospy.set_param('~card_params', card_params)
        rospy.set_param('~block_params', block_params)
        with open(os.path.join(sys.path[0], '../config/camera_cal.yaml'), 'w') as f:
            f.write(s)
    return [True, '']

"""
机械臂上升抬起回调
"""
def up_cb(msg: TriggerRequest): 
    try:
        stepper = hiwonder.Stepper(1)
    except:
        stepper = None
    try:
        chassis = hiwonder.MecanumChassis()
    except:
        chassis = None
    # 根据不同形态设置不同的高度
    if stepper:
        jetmax.go_home(2,2)
    elif chassis:
        jetmax.go_home(2,3)
    else:
        jetmax.go_home(2)

    return [True, '']

"""
机械臂下降回调
"""
def down_cb(msg):
    try:
        stepper = hiwonder.Stepper(1)
    except:
        stepper = None
    try:
        chassis = hiwonder.MecanumChassis()
    except:
        chassis = None
    # 根据不同形态设置不同的高度
    if stepper:
        dn = 160
    elif chassis:
        dn = 190
    else:
        dn = 110

    jetmax_pub.publish(SetJetMax(
        x=hiwonder.JetMax.ORIGIN[0],
        y=hiwonder.JetMax.ORIGIN[1],
        z=hiwonder.JetMax.ORIGIN[2] - dn,
        duration=2
    ))

    return [True, '']

"""
订阅相机内参发布topic的回调
将收到的内参保存到变量中
"""
def camera_cb(msg):
    new_k = np.asarray(msg.K).reshape((3, 3))
    new_d = np.asarray(msg.D)
    new_k = cv2.getOptimalNewCameraMatrix(new_k, new_d, (640, 480), 0, (640, 480))[0]
    with state.lock:
        state.K = new_k
        state.D = new_d


if __name__ == '__main__':
    state = State()
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    rospy.sleep(0.2)
    # 订阅相机图像
    state.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, image_callback)
    # 机械臂控制topic
    jetmax_pub = rospy.Publisher('/jetmax/command', SetJetMax, queue_size=1)
    # 画面发布图像
    image_pub = rospy.Publisher('/%s/image_result' % ROS_NODE_NAME, Image, queue_size=1)  # register image publisher
    # 相机参数topic
    camera_info_sub = rospy.Subscriber('/usb_cam/camera_info', CameraInfo, camera_cb, queue_size=1)
    # 注册各个服务
    enter_srv = rospy.Service('/%s/enter' % ROS_NODE_NAME, Trigger, enter_func)
    exit_srv = rospy.Service('/%s/exit' % ROS_NODE_NAME, Trigger, exit_func)
    running_srv = rospy.Service('/%s/set_running' % ROS_NODE_NAME, SetBool, set_running)
    up_srv = rospy.Service('/%s/up' % ROS_NODE_NAME, Trigger, up_cb)
    down_srv = rospy.Service('/%s/down' % ROS_NODE_NAME, Trigger, down_cb)
    save_srv = rospy.Service('/%s/save' % ROS_NODE_NAME, Trigger, save_cb)
    heartbeat_srv = rospy.Service('/%s/heartbeat' % ROS_NODE_NAME, SetBool, heartbeat_srv_cb)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        sys.exit(0)
