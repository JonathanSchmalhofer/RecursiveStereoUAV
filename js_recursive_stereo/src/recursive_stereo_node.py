#!/usr/bin/env python
import cv2

import rospy
from std_msgs.msg import String
from RecursiveStereoPackage.RecursiveStereo import RecursiveStereo
import os
import rospkg

def RecursiveStereoNode():
    pub = rospy.Publisher('pointcloud', String, queue_size=1)
    rospy.init_node('recursive_stereo_node', anonymous=True)

    algorithm = RecursiveStereo()
    algorithm.Step()
    rospy.loginfo("Loading Images")

    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    image_left  = cv2.imread(rospack.get_path('js_recursive_stereo') + '/resources/left_0000000000.png')
    image_right = cv2.imread(rospack.get_path('js_recursive_stereo') + '/resources/right_0000000000.png')
    algorithm.left_image  = image_left
    algorithm.right_image = image_right
    algorithm.Step()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        RecursiveStereoNode()
    except rospy.ROSInterruptException:
        pass
