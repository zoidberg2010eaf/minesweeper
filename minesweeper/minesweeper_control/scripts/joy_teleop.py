#!/usr/bin/env python
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import rospy


class Controller(object):
    def __init__(self):
        rospy.init_node("controller", anonymous=False)
        rospy.Subscriber("joy", Joy, self.callback)
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.velocity = Twist()
        self._linear_axis = 0.0
        self._angular_axis = 0.0
        self._max_linear_speed = 0.10
        self._max_angular_speed = 0.05
        self._max_linear_axis = 0.0
        self._max_angular_axis = 0.0
        self._linear_lock = False
        self._angular_lock = False

    def callback(self, data):
        self._linear_axis = data.axes[1]
        self._angular_axis = data.axes[3] * -1.0
        self._max_linear_axis = data.axes[7]
        self._max_angular_axis = data.axes[6] * -1.0

    def publisher(self):
        while not rospy.is_shutdown():
            if self._max_linear_axis != 0.0:
                if self._max_linear_speed <= 1.50 and self._max_linear_speed >= 0.10:
                    if not self._linear_lock:
                        self._max_linear_speed += self._max_linear_axis * 0.10
                        self._linear_lock = True
                else:
                    if self._max_linear_speed > 1.50:
                        self._max_linear_speed = 1.50
                    elif self._max_linear_speed < 0.10:
                        self._max_linear_speed = 0.10
            else:
                self._linear_lock = False

            if self._max_angular_axis != 0.0:
                if self._max_angular_speed <= 1.00 and self._max_angular_speed >= 0.05:
                    if not self._angular_lock:
                        self._max_angular_speed += self._max_angular_axis * 0.05
                        self._angular_lock = True
                else:
                    if self._max_angular_speed > 1.00:
                        self._max_angular_speed = 1.00
                    elif self._max_angular_speed < 0.05:
                        self._max_angular_speed = 0.05
            else:
                self._angular_lock = False

            self.velocity.linear.x = self._linear_axis * self._max_linear_speed
            self.velocity.angular.z = self._angular_axis * self._max_angular_speed
            self.pub.publish(self.velocity)
            self.rate.sleep()


def main():
    xbox = Controller()
    xbox.publisher()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
