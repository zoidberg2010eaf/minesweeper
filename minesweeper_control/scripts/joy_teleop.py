#!/usr/bin/env python
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray
import rospy
import math


class Controller(object):
    def __init__(self):
        rospy.init_node("controller", anonymous=False)
        self._sub = rospy.Subscriber("joy", Joy, self.callback)
        self._rate = rospy.Rate(50)
        self._pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self._velocity = Twist()
        self._left_stick_y = 0.0
        self._left_stick_x = 0.0
        self._right_stick_x = 0.0
        self._r2 = 0.0
        self._l2 = 0.0
        self._r22 = 0.0
        self._l22 = 0.0
        self._profile_2_linear_axis = 0.0
        self._max_linear_speed = 0.10
        self._max_angular_speed = 0.05
        self._dpad_y = 0.0
        self._dpad_x = 0.0
        self._inverted_right_stick_x = 1  # for profile 1
        self._inverted_left_stick_x = 1  # for profile 2
        self._r3 = 1
        self._l3 = 1
        self._share_button = 0
        self._cross = 0
        self._profile = 1
        self._linear_lock = False
        self._inverted_angular_lock = False
        self._angular_lock = False
        self._share_lock = False
        self._profile_2_linear_lock = True

        # Remove in case of Quadrature encoder
        self._sign_pub = rospy.Publisher("sign", Int8MultiArray, queue_size=1)
        self._l_sign = 1
        self._r_sign = 1
        self._signs = Int8MultiArray()

    def callback(self, data):
        self._left_stick_y = data.axes[1]
        self._left_stick_x = data.axes[0] * self._inverted_left_stick_x
        self._right_stick_x = data.axes[3] * self._inverted_right_stick_x
        self._r2 = data.axes[5]
        self._l2 = data.axes[2]
        self._dpad_y = data.axes[7]
        self._dpad_x = data.axes[6] * -1
        self._r3 = data.buttons[10]
        self._l3 = data.buttons[9]
        self._share_button = data.buttons[6]
        self._cross = data.buttons[2]

    def profile_1(self):
        self.speed_limits()
        if -0.04 <= self._left_stick_y <= 0.04:
            self._left_stick_y = 0.0
        if -0.05 <= self._right_stick_x <= 0.05:
            self._right_stick_x = 0.0
        if self._r3 != 0:
            if not self._inverted_angular_lock:
                self._inverted_right_stick_x = self._inverted_right_stick_x * -1
                self._inverted_angular_lock = True
        else:
            self._inverted_angular_lock = False

        self._velocity.linear.x = self._left_stick_y * self._max_linear_speed
        self._velocity.angular.z = self._right_stick_x * self._max_angular_speed

    def profile_2(self):
        self.speed_limits()
        self._r22 = 1.0 + (((self._r2 - (-1.0))/(1.0 - (-1.0))) * (0.0 - 1.0))
        self._l22 = -1.0 + (((self._l2 - (-1.0)) / (1.0 - (-1.0))) * (1.0 - 0.0))
        if -0.05 <= self._left_stick_x <= 0.05:
            self._left_stick_x = 0.0
        if self._l3 != 0:
            if not self._inverted_angular_lock:
                self._inverted_left_stick_x = self._inverted_left_stick_x * -1
                self._inverted_angular_lock = True
        else:
            self._inverted_angular_lock = False
        if self._profile_2_linear_lock:
            if self._r2 != 0.0:
                self._profile_2_linear_axis = self._r22
            if self._l2 != 0.0:
                self._profile_2_linear_axis = self._l22
        if self._r2 == 1.0 and self._l2:
            self._profile_2_linear_lock = False
        if not self._profile_2_linear_lock:
            self._profile_2_linear_axis = self._l22 + self._r22

        self._velocity.linear.x = self._profile_2_linear_axis * self._max_linear_speed
        self._velocity.angular.z = self._left_stick_x * self._max_angular_speed

    def speed_limits(self):
        if self._dpad_y != 0.0:
            if 0.10 <= self._max_linear_speed <= 1.50:
                if not self._linear_lock:
                    self._max_linear_speed += self._dpad_y * 0.10
                    self._linear_lock = True
            else:
                if self._max_linear_speed > 1.50:
                    self._max_linear_speed = 1.50
                elif self._max_linear_speed < 0.10:
                    self._max_linear_speed = 0.10
        else:
            self._linear_lock = False
        if self._dpad_x != 0.0:
            if 0.05 <= self._max_angular_speed <= 1.00:
                if not self._angular_lock:
                    self._max_angular_speed += self._dpad_x * 0.05
                    self._angular_lock = True
            else:
                if self._max_angular_speed > 1.00:
                    self._max_angular_speed = 1.00
                elif self._max_angular_speed < 0.05:
                    self._max_angular_speed = 0.05
        else:
            self._angular_lock = False

    def stop(self):
        if self._cross == 1:
            self._velocity.linear.x = 0.0
            self._velocity.angular.z = 0.0

    def publisher(self):
        while not rospy.is_shutdown():
            if self._share_button == 1:
                if not self._share_lock:
                    if self._profile == 1:
                        self._profile = 2
                    elif self._profile == 2:
                        self._profile = 1
                    self._velocity.linear.x = 0.0
                    self._velocity.angular.z = 0.0
                    self._share_lock = True
            else:
                self._share_lock = False

            if self._profile == 1:
                self.profile_1()
            elif self._profile == 2:
                self.profile_2()
            self.stop()
            self._pub.publish(self._velocity)
            self.sign_publisher()
            self._rate.sleep()

    # Remove in case of Quadrature encoder
    def sign_publisher(self):
        if not self._velocity.angular.z:
            self._l_sign = math.copysign(1, self._velocity.linear.x)
            self._r_sign = math.copysign(1, self._velocity.linear.x)
        elif self._velocity.angular.z > 0.0:
            self._l_sign = math.copysign(1, self._velocity.linear.x)
            self._r_sign = math.copysign(1, self._velocity.linear.x) * -1.0
        elif self._velocity.angular.z < 0.0:
            self._l_sign = math.copysign(1, self._velocity.linear.x) * -1.0
            self._r_sign = math.copysign(1, self._velocity.linear.x)
        self._signs.data = (self._l_sign, self._r_sign)
        self._sign_pub.publish(self._signs)


def main():
    xbox = Controller()
    xbox.publisher()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
