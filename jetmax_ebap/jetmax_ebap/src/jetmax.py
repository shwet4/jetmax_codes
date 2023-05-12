import rospy
import time
import math
import threading
import hiwonder
import justkinematics

L0 = 84.4
L1 = 8.14
L2 = 128.4
L3 = 138.0
L4 = 16.8


class Sucker:
    def __init__(self):
        self.__state = False  # False for release, True for suck
        self.__timer = threading.Timer(0.1, self.release_cb)
        self.__timer.start()

    def suck(self):
        if self.__timer:
            self.__timer.cancel()
            self.__timer = None
        hiwonder.motor2.set_speed(0)  # Close the vent valve
        hiwonder.motor1.set_speed(100)  # Turn on the air pump
        self.__state = True

    def release(self, duration=0.5):
        if self.__timer:
            self.__timer.cancel()
            self.__timer = None
        hiwonder.motor1.set_speed(0)  # Turn off the air pump
        hiwonder.motor2.set_speed(100)  # Open the vent valve
        self.__timer = threading.Timer(duration, self.release_cb)
        self.__timer.start()
        self.__state = False

    def release_cb(self):
        self.__timer = None
        hiwonder.motor1.set_speed(0)
        hiwonder.motor2.set_speed(0)

    def get_state(self):
        return self.__state

    def set_state(self, new_state):
        if new_state:
            self.suck()
        else:
            self.release()

class JetMax:

    ORIGIN = 0, -(L1 + L3 + L4), (L0 + L2)

    def __init__(self, origin=ORIGIN):
        self.__lock = threading.RLock()
        self.origin = origin
        self.position = self.origin
        self.joints = 120, 90, 0
        self.servos = 500, 500, 500  # [servo id 1, servo id 2, servo id 3]
        self.last_position = self.position
        self.distance = 0
        self.duration = 0
        self.time_stamp = time.time()
        self.L4 = L4

    def set_servo_in_range(self, servo_id, p, duration):
        if servo_id == 3 and p < 470:
            p = 470
        if servo_id == 2 and p > 700:
            p = 700

        hiwonder.serial_servo.set_position(servo_id, int(p), duration)
        return True

    def set_position(self, position, duration):
        duration = int(duration * 1000)
        x, y, z = position
        if z > 225:
            z = 225
        if math.sqrt(x ** 2 + y ** 2) < 50:
            return None
        angles = justkinematics.inverse(self.L4, (x, y, z))
        pulses = justkinematics.deg_to_pulse(angles)
        with self.__lock:
            for i in range(3):
                ret = self.set_servo_in_range(i + 1, pulses[i], duration)
                if not ret:
                    raise ValueError("{} Out of limit range".format(pulses[i]))
            self.servos = pulses
            self.joints = angles
            self.position = x, y, z

    def set_position_with_speed(self, position, speed):
        with self.__lock:
            old_position = self.position
            distance = math.sqrt(sum([(position[i] - old_position[i]) ** 2 for i in range(0, 3)]))
            duration = distance / max(speed, 0.001)
            self.distance = distance
            self.last_position = self.position
            self.duration = duration
            self.time_stamp = time.time()
            self.set_position(position, duration)

    def set_position_relatively(self, values, duration):
        with self.__lock:
            x, y, z = self.position
            x_v, y_v, z_v = values
            x += x_v
            y += y_v
            z += z_v
            return self.set_position((x, y, z), duration)

    def set_servo(self, servo_id, pulse, duration):
        if not 0 < servo_id < 4:
            raise ValueError("Invalid servo id:{}".format(servo_id))
        pulse = 0 if pulse < 0 else pulse
        pulse = 1000 if pulse > 1000 else pulse
        with self.__lock:
            servos = list(self.servos)
            servos[servo_id - 1] = pulse
            joints = justkinematics.pulse_to_deg(servos)
            new_position = justkinematics.forward(self.L4, joints)
            self.set_position(new_position, duration)

    def set_servo_with_speed(self, servo_id, pulse, speed):
        if not 0 < servo_id < 4:
            raise ValueError("Invalid servo id:{}".format(servo_id))
        pulse = 0 if pulse < 0 else pulse
        pulse = 1000 if pulse > 1000 else pulse
        with self.__lock:
            servos = list(self.servos)
            servos[servo_id - 1] = pulse
            joints = justkinematics.pulse_to_deg(servos)
            new_position = justkinematics.forward(self.L4, joints)
            self.set_position_with_speed(new_position, speed)

    def set_servo_relatively(self, servo_id, value, duration):
        if not 0 < servo_id < 4:
            raise ValueError("Invalid servo id:{}".format(servo_id))
        index = servo_id - 1
        with self.__lock:
            pulse = self.servos[index]
            pulse += value
            return self.set_servo(servo_id, pulse, duration)

    def set_joint(self, joint_id, angle, duration):
        if not 0 < joint_id < 4:
            raise ValueError("Invalid joint id:{}".format(joint_id))
        with self.__lock:
            angles = list(self.joints)
            angles[joint_id - 1] = angle
            servos = justkinematics.deg_to_pulse(angles)
            return self.set_servo(joint_id, servos[joint_id - 1], duration)

    def set_joint_relatively(self, joint_id, value, duration):
        if not 0 < joint_id < 4:
            raise ValueError("Invalid joint id:{}".format(joint_id))
        with self.__lock:
            angles = list(self.joints)
            angles[joint_id - 1] += value
            servos = justkinematics.deg_to_pulse(angles)
            return self.set_servo(joint_id, servos[joint_id - 1], duration)

    def go_home(self, duration=2, pattern = 1):
        if pattern == 1:
            self.set_position(self.origin, duration)
        elif pattern == 2:
            x,y,z = self.origin
            self.set_position((x,y,z-60), duration)
        elif pattern == 3:
            x,y,z = self.origin
            self.set_position((x,y,z-90), duration)


