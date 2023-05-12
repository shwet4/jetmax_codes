#!/usr/bin/env python3
import sys
import cv2
import math
import rospy
import queue
import signal
import hiwonder
import threading
import numpy as np
import mediapipe as mp
import numpy.linalg as LA
from sensor_msgs.msg import Image
from PyQt5.QtCore import Qt, QPoint, QRect
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QPainter, QPixmap, QColor,QPen,QFont
from matplotlib import pyplot as plt

# 指尖控制

ROS_NODE_NAME = "fingertip_control"

class HandGesture:
    def __init__(self):
        self.moving_color = None
        self.target_colors = {}
        self.lock = threading.RLock
        self.position = 0, 0, 0
        self.runner = None
        self.count = 0
        self.gesture_str = ''
        self.servo_x = 500
        self.last_coord = (0,0)
        self.locus_coord = []
        self.pid_x = hiwonder.PID(0.10, 0, 0.01)
        self.pid_z = hiwonder.PID(0.13, 0, 0.015)

# 求两个向量夹角
def vector_2d_angle(v1, v2):
    """
       Solve the angle between two vector
    """
    v1_x = v1[0]
    v1_y = v1[1]
    v2_x = v2[0]
    v2_y = v2[1]
    try:
        angle_ = math.degrees(math.acos(
            (v1_x * v2_x + v1_y * v2_y) / (((v1_x ** 2 + v1_y ** 2) ** 0.5) * ((v2_x ** 2 + v2_y ** 2) ** 0.5))))
    except:
        angle_ = 65535.
    if angle_ > 180.:
        angle_ = 65535.
    return angle_


def hand_angle(hand_):
    """
        Obtain the angle of the corresponding hand-related vector, and determine the gesture according to the angle
    """
    angle_list = []
    # ---------------------------- thumb
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[2][0])), (int(hand_[0][1]) - int(hand_[2][1]))),
        ((int(hand_[3][0]) - int(hand_[4][0])), (int(hand_[3][1]) - int(hand_[4][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- index
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[6][0])), (int(hand_[0][1]) - int(hand_[6][1]))),
        ((int(hand_[7][0]) - int(hand_[8][0])), (int(hand_[7][1]) - int(hand_[8][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- middle
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[10][0])), (int(hand_[0][1]) - int(hand_[10][1]))),
        ((int(hand_[11][0]) - int(hand_[12][0])), (int(hand_[11][1]) - int(hand_[12][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- ring
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[14][0])), (int(hand_[0][1]) - int(hand_[14][1]))),
        ((int(hand_[15][0]) - int(hand_[16][0])), (int(hand_[15][1]) - int(hand_[16][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- pink
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[18][0])), (int(hand_[0][1]) - int(hand_[18][1]))),
        ((int(hand_[19][0]) - int(hand_[20][0])), (int(hand_[19][1]) - int(hand_[20][1])))
    )
    angle_list.append(angle_)
    return angle_list


def h_gesture(angle_list):
    """
        Use the angle of the corresponding hand-related to define the gesture

    """
    thr_angle = 65.
    thr_angle_thumb = 53.
    thr_angle_s = 49.
    gesture_str = None
    if 65535. not in angle_list:
        if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
                angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
            gesture_str = "fist"
        elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
                angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
            gesture_str = "hand_heart"
        elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
                angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
            gesture_str = "nico-nico-ni"
        elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
                angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
            gesture_str = "hand_heart"
        elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
                angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
            gesture_str = "one"
        elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
                angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
            gesture_str = "two"
        elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
                angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
            gesture_str = "three"
        elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] < thr_angle_s) and (
                angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
            gesture_str = "OK"
        elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
                angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
            gesture_str = "four"
        elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
                angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
            gesture_str = "five"
        elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
                angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
            gesture_str = "six"
        else:
            "none"

    return gesture_str

def curvature(x,y):
    """
    input  : the coordinate of the three point
    output : the curvature and norm direction
    """
    t_a = LA.norm([x[1]-x[0],y[1]-y[0]])
    t_b = LA.norm([x[2]-x[1],y[2]-y[1]])
    
    M = np.array([
        [1, -t_a, t_a**2],
        [1, 0,    0     ],
        [1,  t_b, t_b**2]])

    a = np.matmul(LA.inv(M),x)
    b = np.matmul(LA.inv(M),y)
    kappa = 2*(a[2]*b[1]-b[2]*a[1])/(a[1]**2.+b[1]**2.)**(1.5)

    return kappa, [b[1],-a[1]]/np.sqrt(a[1]**2.+b[1]**2.)

def move(*args):
    x = []
    y = []
    l = len(args)
    if l >= 5:
        for i in range(0,l,int(l/2)-1):
            x.append(args[i][0])
            y.append(args[i][1])

        if len(x) >= 3:
            try:
                k,b = curvature(x,y) #计算指尖轨迹曲率
                print(k,b)
            except:
                state.runner = None
                return

            if abs(k) > 0.03 and abs(k) < 1: # 指尖轨迹为弧形
                if k > 0: # 逆时针转
                    chassis.set_velocity(0,0,30)
                    rospy.sleep(5.8)
                    chassis.set_velocity(0,0,0)
                else:  # 顺时针转
                    chassis.set_velocity(0,0,-30)
                    rospy.sleep(5.8)
                    chassis.set_velocity(0,0,0)

            elif abs(k) < 0.01: # 指尖轨迹为直线
                dx = x[0] - x[2]
                dy = y[0] - y[2]
                if abs(dx) > 50 > abs(dy):
                    if dx > 0:  # 向左横移
                        chassis.set_velocity(60,0,0)
                        rospy.sleep(2)
                        chassis.set_velocity(0,0,0)
                    else:   # 向右横移
                        chassis.set_velocity(60,180,0)
                        rospy.sleep(2)
                        chassis.set_velocity(0,0,0)
 
                elif abs(dy) > 50 > abs(dx):
                    if dy > 0: # 向前移动
                        chassis.set_velocity(60,90,0)
                        rospy.sleep(2)
                        chassis.set_velocity(0,0,0)
                    else:  # 向后移动
                        chassis.set_velocity(60,270,0)
                        rospy.sleep(2)
                        chassis.set_velocity(0,0,0)   
                 
    state.runner = None

# 图像处理函数
def image_proc(img):
    ret_img = np.copy(img)
    if state.runner is not None:
        return ret_img

    results = hands.process(img)
    
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(ret_img, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            hand_local = [(landmark.x * ret_img.shape[1], landmark.y * ret_img.shape[0]) for landmark in hand_landmarks.landmark]
            if hand_local:
                angle_list = hand_angle(hand_local)
                gesture_str = h_gesture(angle_list)
                cv2.putText(ret_img, gesture_str, (10, 50), 0, 1.5, (100, 200, 255), 3)
                if gesture_str == 'one':
                    x, y = hand_local[8]
                    x1, y1 = state.last_coord
                    if abs(x-x1) >= 5 or abs(y-y1) >= 5:
                       if x1 == 0 and y1 == 0:
                           x1, y1 = x, y
                       state.last_coord = hand_local[8]
                       state.locus_coord.append((int(x),int(y)))
       
                elif gesture_str == 'five':
                    if state.locus_coord:
                        #print(state.locus_coord)
                        if state.runner is None:
                            state.runner = threading.Thread(target=move, args=state.locus_coord, daemon=True)
                            state.runner.start()
                        state.locus_coord.clear()

    if state.locus_coord is not None: 
        for i in range(1,len(state.locus_coord)):
            cv2.line(ret_img, state.locus_coord[i-1], state.locus_coord[i], (255,0,100), 5) 

    else:
        state.count = 0

    return ret_img

# 图像转换和显示
def image_proc_b():
    image = image_queue.get(block=True)
    image = image_proc(image)
    image = cv2.resize(image, (640, 480))
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.imshow(ROS_NODE_NAME, image)
    cv2.waitKey(1)

# 图像回调函数
def image_callback(ros_image):
    try:
        image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        image = cv2.flip(image, 1)
        image_queue.put_nowait(image)
    except:
        pass

# 停止运行函数
def shutdown(signum, frame):
    print('shutdown')
    sys.exit()    

signal.signal(signal.SIGINT, shutdown)

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    rospy.sleep(0.2)
    state = HandGesture()
    image_queue = queue.Queue(maxsize=1)
    mp_drawing = mp.solutions.drawing_utils
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.70,
        min_tracking_confidence=0.1)

    chassis = hiwonder.MecanumChassis()
    jetmax = hiwonder.JetMax()
    jetmax.go_home()
    rospy.sleep(1)
    hiwonder.motor2.set_speed(0)
    image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, image_callback, queue_size=1)

    while True:
     try:
         image_proc_b()
     except Exception as e:
         print(e)
         sys.exit()
