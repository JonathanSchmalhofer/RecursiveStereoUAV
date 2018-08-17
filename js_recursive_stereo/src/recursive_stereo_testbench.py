#!/usr/bin/env python
import os
import cv2
from RecursiveStereoPackage.RecursiveStereo import RecursiveStereo

if __name__ == '__main__':
    algorithm = RecursiveStereo()
    # Set parameters
    algorithm.c_u = 399.5661
    algorithm.c_v = 295.6579
    algorithm.b   = 0.1400
    algorithm.f   = 573.2981
    algorithm.blockmatching_blocksize           = 10
    algorithm.blockmatching_window_size         = 9
    algorithm.blockmatching_minimum_disparities = 1
    algorithm.blockmatching_maximum_disparities = 65
    algorithm.export_pcl       = True
    algorithm.enable_recursive = True
    algorithm.verbose          = True
    
    # Run
    algorithm.Step()
    color_image = cv2.imread('../resources/AirSimNeighbourhood/images/left_00012.png')
    left_image  = cv2.imread('../resources/AirSimNeighbourhood/images/left_00012.png')
    right_image = cv2.imread('../resources/AirSimNeighbourhood/images/right_00012.png')
    algorithm.left_image  = left_image
    algorithm.right_image = right_image
    algorithm.color_image = color_image
    algorithm.pcl_filename = 'step_1.ply'
    algorithm.Step()
    
    # Set a pose
    new_pose = algorithm.EmptyPose()
    new_pose['position']['x'] = 10
    algorithm.pose = new_pose
    algorithm.pcl_filename = 'step_2.ply'    
    algorithm.Step()