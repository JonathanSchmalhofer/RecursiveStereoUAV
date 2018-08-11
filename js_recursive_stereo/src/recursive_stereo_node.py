#!/usr/bin/env python
import cv2

import rospy
import rospkg
from std_msgs.msg      import String, Header
from sensor_msgs.msg   import Image
from geometry_msgs.msg import Point
from sensor_msgs.msg   import PointCloud
from cv_bridge         import CvBridge, CvBridgeError

from RecursiveStereoPackage.RecursiveStereo import RecursiveStereo


class RecursiveStereoNode:
    def __init__(self):
        self.rospack          = rospkg.RosPack() # get an instance of RosPack with the default search paths
        self.subscriber_left  = rospy.Subscriber('/airsim/left/image_raw',  Image, self.CallbackLeft, queue_size = 1)
        self.subscriber_right = rospy.Subscriber('/airsim/right/image_raw', Image, self.CallbackRight, queue_size = 1)
        self.publisher        = rospy.Publisher('/airsim/pointcloud', PointCloud, queue_size = 1)
        self.cv_bridge        = CvBridge()
        self.sequence_left    = None
        self.sequence_right   = None
        
        # Recursive Stereo
        self.algorithm = RecursiveStereo()
        
        # Set parameters
        self.algorithm.c_u = rospy.get_param('/recursivestereo/parameters/c_u', 609.5593)
        self.algorithm.c_v = rospy.get_param('/recursivestereo/parameters/c_v', 172.8840)
        self.algorithm.b   = rospy.get_param('/recursivestereo/parameters/b',     0.5572)
        self.algorithm.f   = rospy.get_param('/recursivestereo/parameters/f',   721.5577)
        self.algorithm.blockmatching_blocksize           = rospy.get_param('/recursivestereo/parameters/blockmatching_blocksize',            10)
        self.algorithm.blockmatching_window_size         = rospy.get_param('/recursivestereo/parameters/blockmatching_window_size',           9)
        self.algorithm.blockmatching_minimum_disparities = rospy.get_param('/recursivestereo/parameters/blockmatching_minimum_disparities',   1)
        self.algorithm.blockmatching_maximum_disparities = rospy.get_param('/recursivestereo/parameters/blockmatching_maximum_disparities',  65)
        self.algorithm.export_pcl       = True
        self.algorithm.enable_recursive = True
    
    def CallbackLeft(self, image):
        cv_image_left = None
        try:
            cv_image_left = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.algorithm.left_image  = cv_image_left
        self.sequence_left         = image.header.seq
        self.CallbackCalculate()
    
    def CallbackRight(self, image):
        cv_image_right = None
        try:
            cv_image_right = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.algorithm.right_image  = cv_image_right
        self.sequence_right         = image.header.seq
        self.CallbackCalculate()
    
    def CallbackCalculate(self):
        if ((self.sequence_left is None) or (self.sequence_right is None)):
            return
        if self.sequence_left == self.sequence_right:
            self.algorithm.pcl_filename = 'airsim_pcl_seq_{}.ply'.format(self.sequence_left)
            self.algorithm.Step()
            pointcloud = PointCloud()
            print('Calculating')
            print(self.algorithm.pcl.shape)
            print(len(self.algorithm.pcl))
            print(type(pointcloud.points))
            print(len(pointcloud.points))
            for i in range(len(self.algorithm.pcl)):
                 pointcloud.points.append(Point(self.algorithm.pcl[i,0], 
                                                self.algorithm.pcl[i,1], 
                                                self.algorithm.pcl[i,2]))
            self.publisher.publish(pointcloud)
        else:
            print('Sequence of left image and rigt image do not match (yet)')
            print('   Seq(Left)  = {}'.format(self.sequence_left))
            print('   Seq(Right) = {}'.format(self.sequence_right))
    
if __name__ == '__main__':
    rospy.init_node('recursive_stereo_node', anonymous=True)
    node = RecursiveStereoNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
