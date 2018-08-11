#!/usr/bin/env python
import cv2

import rospy
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from RecursiveStereoPackage.RecursiveStereo import RecursiveStereo


class RecursiveStereoNode:
    def __init__(self):
        self.publisher       = rospy.Publisher('pointcloud', String, queue_size=1)
        self.rospack         = rospkg.RosPack() # get an instance of RosPack with the default search paths
        self.subscriber_left = rospy.Subscriber('/airsim/left/image_raw', Image, self.Callback)
        self.bridge = CvBridge()
        
        # Recursive Stereo
        self.algorithm = RecursiveStereo()
        
        # Set parameters
        self.algorithm.c_u = rospy.get_param('/recursivestereo/parameters/c_u', 609.5593)
        self.algorithm.c_v = rospy.get_param('/recursivestereo/parameters/c_v', 172.8840)
        self.algorithm.b   = rospy.get_param('/recursivestereo/parameters/b',     0.5572)
        self.algorithm.f   = rospy.get_param('/recursivestereo/parameters/f',   721.5577)
        self.algorithm.blockmatching_blocksize           = rospy.get_param('/recursivestereo/parameters/blockmatching_blocksize',            15)
        self.algorithm.blockmatching_window_size         = rospy.get_param('/recursivestereo/parameters/blockmatching_window_size',           9)
        self.algorithm.blockmatching_minimum_disparities = rospy.get_param('/recursivestereo/parameters/blockmatching_minimum_disparities', -64)
        self.algorithm.blockmatching_maximum_disparities = rospy.get_param('/recursivestereo/parameters/blockmatching_maximum_disparities', 128)
        self.algorithm.export_pcl       = False
        self.algorithm.enable_recursive = True
    

        rospy.loginfo("Loading Images")
        image_left  = cv2.imread(rospack.get_path('js_recursive_stereo') + '/resources/left_0000000000.png')
        image_right = cv2.imread(rospack.get_path('js_recursive_stereo') + '/resources/right_0000000000.png')
        self.algorithm.left_image  = image_left
        self.algorithm.right_image = image_right
        #self.algorithm.Step()
    
    def Callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        self.algorithm.Step()
        hello_str = "received an image %s" % rospy.get_time()
        self.publisher.publish(hello_str)
    
if __name__ == '__main__':
    node = RecursiveStereoNode()
    rospy.init_node('recursive_stereo_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
