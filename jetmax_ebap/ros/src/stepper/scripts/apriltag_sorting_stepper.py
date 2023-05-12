#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import math
import time
import yaml
import rospy
import hiwonder
import apriltag
import threading
import numpy as np
from sensor_msgs.msg import Image

ROS_NODE_NAME = "ApriltagSorting_stepper"

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

TARGET_POSITIONS = {
    1: (0, -180, 35, 5200),
    2: (0, -180, 35, 4200),
    3: (0, -180, 35, 3200),
}

IMAGE_PROC_SIZE = 640, 480
TAG_SIZE = 33.30
stepper_position = 0
jetmax = hiwonder.JetMax()
sucker = hiwonder.Sucker()
stepper = hiwonder.Stepper(1)
at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))

class ApriltagSorting:
    def __init__(self):
        self.lock = threading.RLock()
        self.moving_box = None
        self.moving_block = None
        self.index = 0
        self.level = 0
        self.pos_add = 0
        self.tag_id = None
        self.runner = None
        self.moving_count = 0
        self.count = 0
        self.fps_t0 = time.time()
        self.fps = 0
        self.camera_params = None
        self.K = None
        self.R = None
        self.T = None


    def load_camera_params(self):
        self.camera_params = rospy.get_param('/camera_cal/block_params', self.camera_params)
        if self.camera_params is not None:
            self.K = np.array(self.camera_params['K'], dtype=np.float64).reshape(3, 3)
            self.R = np.array(self.camera_params['R'], dtype=np.float64).reshape(3, 1)
            self.T = np.array(self.camera_params['T'], dtype=np.float64).reshape(3, 1)
 
# 现实坐标转换函数
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


def rotation_mtx_to_euler(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

# 移动控制函数
def moving():
    try:
        tag = state.moving_block
        params = np.array([state.K[0][0], state.K[1][1], state.K[0][2], state.K[1][2]])
        pose_mtx, a, b = at_detector.detection_pose(tag, camera_params=params, tag_size=TAG_SIZE)
        angle = rotation_mtx_to_euler(pose_mtx)[2] * (180 / math.pi) 

        cur_x, cur_y, cur_z = jetmax.position # 当前的机械臂位置
        rect_x, rect_y = state.moving_block.center
        p = np.asarray([rect_x, rect_y]).reshape((1, 1, 2))
        w = camera_to_world(state.K, state.R, state.T, p)[0][0] #转换成现实距离
        x, y, _ = w
        print(w)
        if angle < -45:  # ccw -45 ~ -90
            angle = -(-90 - angle)
        if angle > 45:
            angle = -(90 - angle)

        new_x, new_y = cur_x + x, cur_y + y #目标的现实坐标
        arm_angle = math.atan(new_y / new_x) * 180 / math.pi # 计算偏转角
        if arm_angle > 0:
            arm_angle = (90 - arm_angle)
        elif arm_angle < 0:
            arm_angle = (-90 - arm_angle)
        else:
            pass

        angle = angle + -arm_angle

        dist = math.sqrt(x * x + y * y + 120 * 120) #计算两坐标点的直线距离
        t = dist / 140
        hiwonder.pwm_servo1.set_position(90 + angle, 0.1) #角度补偿
        jetmax.set_position((new_x, new_y, 100), t) # 移动到目标上方
        rospy.sleep(t + 0.1)

        sucker.set_state(True) # 打开气泵
        jetmax.set_position((new_x, new_y, 30), 1) # 吸取目标
        rospy.sleep(1)
        jetmax.set_position((new_x, new_y, 100), 1) # 移动到目标上方
        rospy.sleep(1)

        x, y, z, stepper_position = TARGET_POSITIONS[state.tag_id] # 读取放置坐标


        hiwonder.pwm_servo1.set_position(90, 0.5) #角度回正
        cur_x, cur_y, cur_z = jetmax.position  # 当前的机械臂位置
        t = math.sqrt((cur_x - x) ** 2 + (cur_y - y) ** 2) / 120 # 计算需要的时间
        jetmax.set_position((x, y, 100), t)  # 移动到放置坐标上方
        rospy.sleep(t + 0.1)

        stepper.set_mode(stepper.SLEEP) # 设置滑轨使能
        stepper.goto(stepper_position) # 驱动滑轨移动到放置位置
        stepper_time = stepper_position/1000 # 计算需要的时间
        rospy.sleep(stepper_time)

        jetmax.set_position((x, y, z), 1) # 到放置位置
        rospy.sleep(1)

        sucker.release(3)
        jetmax.set_position((x, y, 100), 0.5)
        rospy.sleep(0.5)

        # Go home
        jetmax.go_home(2,2) # 机械臂回到初始位置 jetmax.go_home(a,b): a:设置运行时间; b: 1 机械臂模式，2 滑轨模式，3 小车底盘模式
        stepper.goto(-stepper_position)  # 滑轨回到初始位置
        stepper_time = stepper_position/1000
        rospy.sleep(stepper_time)
        stepper.set_mode(stepper.EN) # 解除滑轨锁定

    finally:
        
        with state.lock:
            state.tag_id = None
            state.moving_block = None
            state.runner = None


def image_proc(img):
    if state.runner is not None:
        return img
    frame_gray = cv2.cvtColor(np.copy(img), cv2.COLOR_RGB2GRAY)
    tags = at_detector.detect(frame_gray)
    for tag in tags:
        corners = tag.corners.reshape(1, -1, 2).astype(int)
        center = tag.center.astype(int)
        cv2.drawContours(img, corners, -1, (255, 0, 0), 3)
        cv2.circle(img, tuple(center), 5, (255, 255, 0), 10)
        cv2.putText(img, "id:%d" % tag.tag_id,
                    (center[0], center[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    if len(tags) > 0:
        if state.moving_block is None:
            state.moving_block = tags[0]
        else:
            new_tag = tags[0]
            if new_tag.tag_id != state.moving_block.tag_id:
                state.count = 0
            else:
                state.count += 1
                if state.count > 50:
                    state.count = 0
                    state.tag_id = new_tag.tag_id
                    state.runner = threading.Thread(target=moving, daemon=True)
                    state.runner.start()
            state.moving_block = tags[0]
    else:
        state.count = 0
        if state.moving_block is not None:
            state.moving_block = None
    img_h, img_w = img.shape[:2]
    cv2.line(img, (int(img_w / 2 - 10), int(img_h / 2)), (int(img_w / 2 + 10), int(img_h / 2)), (0, 255, 255), 2)
    cv2.line(img, (int(img_w / 2), int(img_h / 2 - 10)), (int(img_w / 2), int(img_h / 2 + 10)), (0, 255, 255), 2)
    return img


def image_callback(ros_image):
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
    frame_result = np.copy(image)
    frame_result = image_proc(frame_result) # apriltag检测
    image_bgr = cv2.cvtColor(frame_result, cv2.COLOR_RGB2BGR)
    cv2.imshow("result", image_bgr)
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    state = ApriltagSorting()
    state.load_camera_params()
    jetmax = hiwonder.JetMax()
    sucker = hiwonder.Sucker()
    stepper = hiwonder.Stepper(1)
    jetmax.go_home(2,2)
    stepper.go_home(True)
    stepper.set_mode(stepper.EN)
    stepper.set_div(stepper.DIV_1_4)
    rospy.sleep(1)
    image_sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, image_callback, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        sys.exit(0)


