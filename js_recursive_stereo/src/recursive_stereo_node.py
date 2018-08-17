#!/usr/bin/env python
import cv2

import rospy
import rospkg
from std_msgs.msg      import String, Header
from sensor_msgs.msg   import Image
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg   import PointCloud
from cv_bridge         import CvBridge, CvBridgeError

from RecursiveStereoPackage.RecursiveStereo import RecursiveStereo


class RecursiveStereoNode:
    def __init__(self):
        self.verbose          = rospy.get_param('/recursivestereo/parameters/verbose', False)
        self.rospack          = rospkg.RosPack() # get an instance of RosPack with the default search paths
        self.subscriber_left  = rospy.Subscriber('/airsim/left/image_raw',  Image, self.CallbackLeft,  queue_size = 1)
        self.subscriber_right = rospy.Subscriber('/airsim/right/image_raw', Image, self.CallbackRight, queue_size = 1)
        self.subscriber_pose  = rospy.Subscriber('/airsim/pose',            Pose,  self.CallbackPose,  queue_size = 1)
        self.publisher        = rospy.Publisher('/airsim/pointcloud', PointCloud, queue_size = 1)
        self.cv_bridge        = CvBridge()
        self.timestamp_left   = None
        self.timestamp_right  = None
        
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
        self.algorithm.export_pcl       = False
        self.algorithm.enable_recursive = True
    
    def VerbosePrint(self, string):
        if self.verbose == True:
            print(string)
    
    def CallbackLeft(self, image):
        cv_image_left = None
        try:
            cv_image_left = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            self.VerbosePrint(e)
        self.algorithm.left_image  = cv_image_left
        self.algorithm.color_image = cv_image_left
        self.timestamp_left        = image.header.stamp.sec*1e9 + image.header.stamp.nsec # ns
        self.CallbackCalculate()
    
    def CallbackRight(self, image):
        cv_image_right = None
        try:
            cv_image_right = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            self.VerbosePrint(e)
        self.algorithm.right_image  = cv_image_right
        self.timestamp_right        = image.header.stamp.sec*1e9 + image.header.stamp.nsec # ns
        self.CallbackCalculate()
    
    def CallbackPose(self, new_pose):
        # Save last pose
        self.algorithm.last_pose = self.algorithm.pose
        # Set new pose
        self.algorithm.pose['position']['x']     = new_pose.position.x
        self.algorithm.pose['position']['y']     = new_pose.position.y
        self.algorithm.pose['position']['z']     = new_pose.position.z
        self.algorithm.pose['orientation']['x']  = new_pose.orientation.x
        self.algorithm.pose['orientation']['y']  = new_pose.orientation.y
        self.algorithm.pose['orientation']['z']  = new_pose.orientation.z
        self.algorithm.pose['orientation']['w']  = new_pose.orientation.w
    
    def CallbackCalculate(self):
        if ((self.timestamp_left is None) or (self.timestamp_right is None)):
            rospy.loginfo("Timestamp(s) missing")
            return
        if self.timestamp_left == self.timestamp_right:
            self.algorithm.pcl_filename = 'airsim_pcl_time_{}.ply'.format(self.timestamp_left)
            self.algorithm.Step()
            pointcloud = PointCloud()
            self.VerbosePrint('Calculating')
            for i in range(len(self.algorithm.pcl)):
                 pointcloud.points.append(Point(self.algorithm.pcl[i,0], 
                                                self.algorithm.pcl[i,1], 
                                                self.algorithm.pcl[i,2]))
            self.publisher.publish(pointcloud)
            rospy.loginfo("Sending PointCloud")
        else:
            self.VerbosePrint('Timestamp of left image and rigt image do not match (yet)')
            self.VerbosePrint('   T(Left)  = {}'.format(self.timestamp_left))
            self.VerbosePrint('   T(Right) = {}'.format(self.timestamp_right))
    
if __name__ == '__main__':
    rospy.init_node('recursive_stereo_node', anonymous=True)
    node = RecursiveStereoNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        self.VerbosePrint("Shutting down")
    cv2.destroyAllWindows()
