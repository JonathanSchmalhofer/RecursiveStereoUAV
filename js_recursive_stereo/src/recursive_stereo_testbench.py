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
    image_color = cv2.imread('../resources/AirSimNeighbourhood/images/left_00012.png')
    image_left  = cv2.imread('../resources/AirSimNeighbourhood/images/left_00012.png')
    image_right = cv2.imread('../resources/AirSimNeighbourhood/images/right_00012.png')
    algorithm.left_image  = image_left
    algorithm.right_image = image_right
    algorithm.color_image = image_color
    algorithm.Step()