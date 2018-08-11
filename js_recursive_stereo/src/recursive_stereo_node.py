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
    rospack = rospkg.RosPack() # get an instance of RosPack with the default search paths
    
    # Recursive Stereo
    algorithm = RecursiveStereo()
    
    # Set parameters
    algorithm.c_u = rospy.get_param('/recursivestereo/parameters/c_u', 609.5593)
    algorithm.c_v = rospy.get_param('/recursivestereo/parameters/c_v', 172.8840)
    algorithm.b   = rospy.get_param('/recursivestereo/parameters/b',     0.5572)
    algorithm.f   = rospy.get_param('/recursivestereo/parameters/f',   721.5577)
    algorithm.blockmatching_blocksize           = rospy.get_param('/recursivestereo/parameters/blockmatching_blocksize',            15)
    algorithm.blockmatching_window_size         = rospy.get_param('/recursivestereo/parameters/blockmatching_window_size',           9)
    algorithm.blockmatching_minimum_disparities = rospy.get_param('/recursivestereo/parameters/blockmatching_minimum_disparities', -64)
    algorithm.blockmatching_maximum_disparities = rospy.get_param('/recursivestereo/parameters/blockmatching_maximum_disparities', 128)
    algorithm.export_pcl       = False
    algorithm.enable_recursive = True
    

    rospy.loginfo("Loading Images")
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
