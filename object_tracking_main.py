#!/usr/bin/env python3
import math
# import rospy
import time
import queue
import threading
import numpy
# from sensor_msgs.msg import Image
# from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
# from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
# # from std_srvs.srv import Empty
# from object_tracking.srv import SetTarget, SetTargetResponse, SetTargetRequest
import cv2
import numpy as np
from jetmax_ebap.build.lib import hiwonder
import sys

# Libraries required for face recognition
sys.path.append("/home/hiwonder/tensorrt_demos")
# from utils.mtcnn import TrtMtcnn
#
# mtcnn = TrtMtcnn() # mtcnn Face recognition network example drawing

image_queue = queue.Queue(maxsize=3) # image queue

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

        #
        # face tracking pid
        self.face_x_pid = hiwonder.PID(0.09, 0.01, 0.015)
        self.face_z_pid = hiwonder.PID(0.16, 0.0, 0.0)

        # color tracking pid
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
    reset variable
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


# Interface objects for recognition and manipulator manipulation
state = ObjectTracking()
jetmax = hiwonder.JetMax()
sucker = hiwonder.Sucker()


# Initialization, the robotic arm returns to the initial position
def init():
    state.reset()
    sucker.set_state(False)
    jetmax.go_home(1)

"""
General Image Processing Functions
Will determine face tracking or color tracking according to the setting status
This function will be called in the while loop instead of being triggered directly by the callback of the ros topic because pycuda requires that the related calls must be in the same thread
So the ros topic callback will only put the received image into a queue and will not process the image directly. image_proc will cycle through the queue
"""
def image_proc():
    # Get new images from the queue
    ros_image = image_queue.get(block=True)
    # convert ros image to opencv image
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
    if state.is_running_face:
        image = face_tracking(image) # face tracking
    elif state.is_running_color:
        image = color_tracking(image) # color tracking
    else:
        pass
    # Calculate frame rate
    toc = time.time()
    curr_fps = 1.0 / (state.tic - toc)
    state.fps = curr_fps if state.fps == 0.0 else (state.fps * 0.95 + curr_fps * 0.05)
    state.tic = toc
    # post result image
    rgb_image = image.tostring()
    ros_image.data = rgb_image
    image_pub.publish(ros_image)


def color_tracking(image):
    org_image = np.copy(image)
    image = cv2.resize(image, (320, 240)) # Scale down to reduce computation
    image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)  # RGB to LAB space
    image = cv2.GaussianBlur(image, (5, 5), 5)

    with state.lock:
        target_color_range = state.target_color_range
        target_color_name = state.target_color_name

    if target_color_range is not None:
        mask = cv2.inRange(image,(127,70,58),(168,235,246))  # Binarization
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # corrosion
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # expand
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find the outline
        contour_area = map(lambda c: (c, math.fabs(cv2.contourArea(c))), contours)  # Calculate the area of each contour
        contour_area = list(filter(lambda c: c[1] > 200, contour_area))  # Eliminate contours that are too small
        circle = None

        if len(contour_area) > 0:
            # The color block was not recognized in the previous frame
            if state.last_color_circle is None:
                contour, area = max(contour_area, key=lambda c_a: c_a[1])
                circle = cv2.minEnclosingCircle(contour) # Minimum circumscribed circle
            else:
            #
            # A color block was identified in the previous frame
                (last_x, last_y), last_r = state.last_color_circle
                # Calculate the center distance between the new color block and the old color block, and if it is too large, it will be considered a misrecognition. Abandon this result when it is not recognized
                circles = map(lambda c: cv2.minEnclosingCircle(c[0]), contour_area) 
                circle_dist = list(map(lambda c: (c, math.sqrt(((c[0][0] - last_x) ** 2) + ((c[0][1] - last_y) ** 2))),
                                       circles))
                circle, dist = min(circle_dist, key=lambda c: c[1])
                if dist < 50:
                    circle = circle

        if circle is not None:
            state.lost_target_count = 0
            (c_x, c_y), c_r = circle
            # Remap the identified coordinates back to the original image size
            c_x = hiwonder.misc.val_map(c_x, 0, 320, 0, 640)
            c_y = hiwonder.misc.val_map(c_y, 0, 240, 0, 480)
            c_r = hiwonder.misc.val_map(c_r, 0, 320, 0, 640)

            # x-axis pid update
            x = c_x - TARGET_PIXEL_X
            if abs(x) > 30: # If the error is within a certain range, it is considered to have reached
                state.color_x_pid.SetPoint = 0
                state.color_x_pid.update(x)
                state.servo_x += state.color_x_pid.output
            else:
                state.color_x_pid.update(0)

            # y-axis pid update
            y = c_y - TARGET_PIXEL_Y
            if abs(y) > 30: # If the error is within a certain range, it is considered to have reached
                state.color_y_pid.SetPoint = 0
                state.color_y_pid.update(y)
                state.servo_y -= state.color_y_pid.output
            else:
                state.color_y_pid.update(0)
            #
            # Limit the range of the steering gear
            if state.servo_y < 350:
                state.servo_y = 350
            if state.servo_y > 650:
                state.servo_y = 650

            #
            # Execute new target position
            jetmax.set_servo(1, int(state.servo_x), duration=0.02)
            jetmax.set_servo(2, int(state.servo_y), duration=0.02)
            color_name = target_color_name.upper()
            # The screen displays the recognition result
            org_image = cv2.circle(org_image, (int(c_x), int(c_y)), int(c_r), hiwonder.COLORS[color_name], 3)
            state.last_color_circle = circle
        else:
            # If it is not recognized more than a certain number of times, the logo will be cleared and re-identified
            state.lost_target_count += 1
            if state.lost_target_count > 15:
                state.lost_target_count = 0
                state.last_color_circle = None
    return org_image

"""
face on screen
And return the corresponding coordinates of the relevant coordinates detected by the face recognition on the original image
"""
def show_faces(img, boxes, landmarks, color):
    new_boxes = []
    new_landmarks = []
    for bb, ll in zip(boxes, landmarks):
        #
        # Map the coordinates back to the original image coordinates
        x1 = int(hiwonder.misc.val_map(bb[0], 0, 400, 0, 640))
        y1 = int(hiwonder.misc.val_map(bb[1], 0, 300, 0, 480))
        x2 = int(hiwonder.misc.val_map(bb[2], 0, 400, 0, 640))
        y2 = int(hiwonder.misc.val_map(bb[3], 0, 300, 0, 480))
        # draw the outline
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        new_boxes.append([x1, y1, x2, y2])
        landmarks_curr_face = []
        if len(landmarks):
            # draw key points
            for j in range(5):
                # Map the coordinates back to the original image coordinates
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
    image = cv2.resize(image, (400, 300)) # Scale down to reduce computation

    boxes, landmarks = mtcnn.detect(image, minsize=40) # 调用 mtcnn 进行人脸识别
    org_img, boxes, landmarks = show_faces(org_img, boxes, landmarks, (0, 255, 0)) # 显示人脸

    # No face was recognized in the previous frame
    if state.tracking_face is None:
        if len(boxes) > 0:  # If a face is recognized, find the face closest to the center of the screen to track
            box = min(boxes, key=lambda b: math.sqrt(((b[2] + b[0]) / 2 - 320) ** 2 + ((b[3] + b[1]) / 2 - 240) ** 2))
            x1, y1, x2, y2 = box
            # Separately draw the faces to be tracked in red
            org_img = cv2.rectangle(org_img, (x1, y1), (x2, y2), (255, 0, 0), 3)
            center_x, center_y = int((x1 + x2) / 2), int((y1 + y2) / 2)
            org_img = cv2.circle(org_img, (center_x, center_y), 2, (0, 255, 0), 4)
            # Record the coordinates of the face to be tracked
            state.tracking_face = center_x, center_y
            state.no_face_count = time.time() + 2
            #
            # cleanup pid controller
            state.face_x_pid.clear()
            state.face_z_pid.clear()
    else: # A face was recognized in the previous frame
        #
        # Calculate the center coordinates of all recognized faces
        centers = [(int((box[2] + box[0]) / 2), int((box[3] + box[1]) / 2), box) for box in boxes]
        get_face = False
        # Find the face closest to the face recognized in the previous frame
        if len(centers) > 0:
            center_x, center_y = state.tracking_face
            org_img = cv2.circle(org_img, (center_x, center_y), 2, (0, 0, 255), 4)
            min_dist_center = min(centers, key=lambda c: math.sqrt((c[0] - center_x) ** 2 + (c[1] - center_y) ** 2))
            new_center_x, new_center_y, box = min_dist_center
            x1, y1, x2, y2 = box
            dist = math.sqrt((new_center_x - center_x) ** 2 + (new_center_y - center_y) ** 2)
            # If the pixel distance is less than a certain value, it is considered to be the same face, and if it exceeds a certain value, it will be regarded as not recognized
            if dist < 150:
                org_img = cv2.rectangle(org_img, (x1, y1), (x2, y2), (255, 0, 0), 3)
                org_img = cv2.circle(org_img, (new_center_x, new_center_y), 2, (255, 0, 0), 4)
                get_face = True
                state.tracking_face = int(new_center_x), int(new_center_y)
                state.no_face_count = time.time() + 2
        # Determine whether the face to be tracked has not been recognized for a certain period of time
        if state.no_face_count < time.time():
            state.tracking_face = None

        # The face to be tracked is recognized in this frame
        if get_face:
            center_x, center_y = state.tracking_face
            x = center_x - 320
            # Update the x-axis pid controller
            if abs(x) > 30:
                state.face_x_pid.SetPoint = 0
                state.face_x_pid.update(x)
                state.servo_x += state.face_x_pid.output
                jetmax.set_servo(1, int(state.servo_x), 0.08)
            else:
                state.face_x_pid.update(0)

            # Update the z-axis pid controller
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
The callback of the camera screen topic
Only received frames will be pushed into the queue
"""
def image_callback(ros_image):
    try:
        image_queue.put_nowait(ros_image)
    except queue.Full:
        pass


"""
start service
The program enters the ready state, ready to run, but does not recognize
"""
def enter_func(msg):
    rospy.loginfo("enter object tracking")
    exit_func(msg)  # Exit once first to simplify the process
    rospy.sleep(0.1)
    init()  # Initialize position and state
    state.target_color_range = None
    state.target_color_name = None
    state.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, image_callback)  # 订阅摄像头画面
    rospy.ServiceProxy('/usb_cam/start_capture', Empty)()
    return [True, '']


def reset():
    exit_func(TriggerRequest())  # Exit once first to simplify the process
    rospy.sleep(0.1)
    init()  # Initialize position and state
    state.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, image_callback)  # 订阅摄像头画面
    rospy.ServiceProxy('/usb_cam/start_capture', Empty)()

"""
quit service
The program unsubscribes from the camera topic
"""
def exit_func(msg):
    rospy.loginfo("exit object tracking")
    state.is_running_color = False
    state.is_running_face = False
    try:
        state.heartbeat_timer.cancel()
    except:
        pass
    try:  # Unsubscribe from the image node so the callback will not be called
        rospy.loginfo('unregister image')
        state.image_sub.unregister()
    except:
        pass
    finally:
        state.image_sub = None
    return TriggerResponse(success=True)


"""
start color tracking
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
start face tracking
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
    Heartbeat callback. A timer will be set, and when the timer arrives, the exit service will be called to exit the gameplay processing
    :params msg: service call parameters, std_srv SetBool
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
Set the target color to track
"""
def set_target_cb(msg: SetTargetRequest):
    # Get a list of color thresholds
    color_ranges = rospy.get_param('/lab_config_manager/color_range_list', {})
    color_ranges={'red':{'max':}}
    rospy.logdebug(color_ranges)
    with state.lock:
        if msg.color_name in color_ranges:
            # Find the target threshold from the threshold list, and set the target
            state.target_color_name = msg.color_name
            state.target_color_range = color_ranges[msg.color_name]
            return [True, '']



        else:
            state.target_color_name = None
            state.target_color_range = None
            return [False, '']


if __name__ == '__main__':
    # initialize node
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    # initializer resource
    init()

    # topic subscription/publish registration, service registration
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
            image_proc() # Loop through images
            if rospy.is_shutdown():
                break
        except KeyboardInterrupt:
            break
