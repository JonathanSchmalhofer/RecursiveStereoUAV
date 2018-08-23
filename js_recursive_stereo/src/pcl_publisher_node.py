#!/usr/bin/env python
import cv2

import rospy
import rospkg
from geometry_msgs.msg import Point
from sensor_msgs.msg   import PointCloud
import os

from RecursiveStereoPackage.RecursiveStereo import RecursiveStereo


class PclPublisherNode:
    def __init__(self):
        self.verbose          = rospy.get_param('/recursivestereo/parameters/verbose', False)
        self.pcl_file         = rospy.get_param('/recursivestereo/parameters/pcl_file', "")
        self.rospack          = rospkg.RosPack() # get an instance of RosPack with the default search paths
        self.publisher        = rospy.Publisher('/airsim/pointcloud', PointCloud, queue_size = 1)
        
        # Recursive Stereo
        self.algorithm  = RecursiveStereo()
	print(self.pcl_file)
        self.pcl        = self.algorithm.ImportPCL(self.pcl_file)
        self.pointcloud = PointCloud()
        for i in range(len(self.pcl)):
             self.pointcloud.points.append(Point(self.pcl[i,0], 
                                                 self.pcl[i,1], 
                                                 self.pcl[i,2]))
    
    def DoPublish(self):
        self.publisher.publish(self.pointcloud)
    
if __name__ == '__main__':
    rospy.init_node('pcl_publisher_node', anonymous=True)
    node = PclPublisherNode()
    
    try:
        rate = rospy.Rate(0.2) # 0.5hz
        while not rospy.is_shutdown():
            node.DoPublish()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
