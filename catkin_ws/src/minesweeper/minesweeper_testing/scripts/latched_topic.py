#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

def main():
    rospy.init_node("node", anonymous=False)
    pub = rospy.Publisher("pub", Float32, latch=True, queue_size=1)
    rate = rospy.Rate(10)
    x = 0
    while not rospy.is_shutdown():
        x += 1.0
        pub.publish(x)
        rospy.loginfo(x)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
