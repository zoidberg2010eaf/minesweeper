#!/usr/bin/env python
import rospy
import tf
import math
import tf2_ros
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import  Vector3

quat = Quaternion()
def callback(data):
    global quat
    quat = data.orientation

def main():
    global  quat
    value = Vector3()
    rospy.init_node("test", anonymous=False)
    pub = rospy.Publisher("topic", Quaternion, queue_size=10)
    rospy.Subscriber("/android/imu", Imu, callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        value = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w], axes="sxyz")
        degrees = Vector3()
        degrees.x = math.degrees(value[0])
        degrees.y = math.degrees(value[1])
        degrees.z = math.degrees(value[2])
        rospy.loginfo(degrees)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
