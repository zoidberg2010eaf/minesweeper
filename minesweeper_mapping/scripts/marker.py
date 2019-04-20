#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Quaternion


class Mine(object):
    def __init__(self):
        rospy.init_node("marker", anonymous=False)
        self._a = 0
        self._b = 0
        self._joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self._odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        self._pub = rospy.Publisher("visualization_marker", Marker, queue_size=100)
        self._rate = rospy.Rate(10)
        self._marker_a = Marker()
        self._marker_b = Marker()
        self._marker_location_a = PoseWithCovariance()
        self._marker_location_b = PoseWithCovariance()
        self._detection_lock_a = False
        self._detection_lock_b = False
        self._id_counter_a = 0
        self._id_counter_b = 0

    def joy_callback(self, data):
        self._a = data.buttons[0]
        self._b = data.buttons[1]
        if self._a == 0:
            self._detection_lock_a = False
        if self._b == 0:
            self._detection_lock_b = False

    def odom_callback(self, data):
        if self._a == 1 and not self._detection_lock_a:
            self._id_counter_a += 1
            self._marker_location_a = data.pose
            self._detection_lock_a = True
        elif self._b and not self._detection_lock_b:
            self._id_counter_b += 1
            self._marker_location_b = data.pose
            self._detection_lock_b = True

    def marker(self):
        while not rospy.is_shutdown():
            self._marker_a.header.frame_id = "/odom"
            self._marker_a.header.stamp = rospy.Time.now()
            self._marker_a.ns = "s_mines"
            self._marker_a.id = self._id_counter_a
            self._marker_a.type = self._marker_a.CUBE
            self._marker_a.action = self._marker_a.ADD
            self._marker_a.pose = self._marker_location_a.pose
            self._marker_a.pose.orientation = Quaternion()
            self._marker_a.pose.orientation.w = 1.0
            self._marker_a.scale.x = 0.50
            self._marker_a.scale.y = 0.50
            self._marker_a.scale.z = 0.10
            self._marker_a.color.r = 1.0
            self._marker_a.color.g = 0.0
            self._marker_a.color.b = 0.0
            self._marker_a.color.a = 1.0
            self._marker_a.lifetime = rospy.Duration()

            self._marker_b.header.frame_id = "/odom"
            self._marker_b.header.stamp = rospy.Time.now()
            self._marker_b.ns = "b_mines"
            self._marker_b.id = self._id_counter_b
            self._marker_b.type = self._marker_b.CYLINDER
            self._marker_b.action = self._marker_b.ADD
            self._marker_b.pose = self._marker_location_b.pose
            self._marker_b.pose.orientation = Quaternion()
            self._marker_b.pose.orientation.w = 1.0
            self._marker_b.scale.x = 0.50
            self._marker_b.scale.y = 0.50
            self._marker_b.scale.z = 0.10
            self._marker_b.color.r = 0.0
            self._marker_b.color.g = 1.0
            self._marker_b.color.b = 0.0
            self._marker_b.color.a = 1.0
            self._marker_b.lifetime = rospy.Duration()
            while self._pub.get_num_connections() < 1:
                pass
            if self._id_counter_a > 0:
                self._pub.publish(self._marker_a)
            if self._id_counter_b > 0:
                self._pub.publish(self._marker_b)
            self._rate.sleep()


def main():
    marker_server = Mine()
    marker_server.marker()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
