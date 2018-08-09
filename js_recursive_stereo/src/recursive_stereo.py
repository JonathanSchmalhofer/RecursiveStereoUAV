#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from RecursiveStereo import RecursiveStereo

def RecursiveStereoNode():
    algorithm = RecursiveStereo()
    algorithm.Step()
    pub = rospy.Publisher('pointcloud', String, queue_size=1)
    rospy.init_node('recursive_stereo', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        RecursiveStereoNode()
    except rospy.ROSInterruptException:
        pass