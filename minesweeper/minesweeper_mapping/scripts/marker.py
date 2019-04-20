#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import  Quaternion

class Mine(object):
    def __init__(self):
        rospy.init_node("marker", anonymous=False)
        self._a = 0
        self._joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self._odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        self._pub = rospy.Publisher("visualization_marker", Marker, queue_size=1)
        self._rate = rospy.Rate(10)
        self._marker = Marker()
        self._marker_location = PoseWithCovariance()
        self._detection_lock = False
        self._id_counter = 0

    def joy_callback(self, data):
        self._a = data.buttons[0]
        if self._a == 0:
            self._detection_lock = False

    def odom_callback(self, data):
        if self._a == 1 and not self._detection_lock:
            self._id_counter += 1
            self._marker_location = data.pose
            self._detection_lock = True

    def marker(self):
        while not rospy.is_shutdown():
            self._marker.header.frame_id = "/odom"
            self._marker.header.stamp = rospy.Time.now()
            self._marker.ns = "mines"
            self._marker.id = self._id_counter
            self._marker.type = self._marker.CUBE
            self._marker.action = self._marker.ADD
            self._marker.pose = self._marker_location.pose
            self._marker.pose.orientation = Quaternion()
            self._marker.pose.orientation.w = 1.0
            self._marker.scale.x = 1.0
            self._marker.scale.y = 1.0
            self._marker.scale.z = 1.0
            self._marker.color.r = 1.0
            self._marker.color.g = 1.0
            self._marker.color.b = 1.0
            self._marker.color.a = 1.0
            self._marker.lifetime = rospy.Duration()
            while self._pub.get_num_connections() < 1:
                pass
            if self._id_counter > 0:
                self._pub.publish(self._marker)
            self._rate.sleep()

def main():
    marker_server = Mine()
    marker_server.marker()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
