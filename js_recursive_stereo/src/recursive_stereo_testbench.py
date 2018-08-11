#!/usr/bin/env python
import os
import cv2
from RecursiveStereoPackage.RecursiveStereo import RecursiveStereo

if __name__ == '__main__':
    algorithm = RecursiveStereo()
    # Set parameters
    algorithm.c_u = 609.5593
    algorithm.c_v = 172.8540
    algorithm.b   = 0.5372
    algorithm.f   = 721.5377
    algorithm.blockmatching_blocksize           = 10
    algorithm.blockmatching_window_size         = 9
    algorithm.blockmatching_minimum_disparities = 1
    algorithm.blockmatching_maximum_disparities = 65
    algorithm.export_pcl       = True
    algorithm.enable_recursive = True
    
    # Run
    algorithm.Step()
    image_color = cv2.imread('../resources/color_0000000000.png')
    image_left  = cv2.imread('../resources/left_0000000000.png')
    image_right = cv2.imread('../resources/right_0000000000.png')
    algorithm.left_image  = image_left
    algorithm.right_image = image_right
    algorithm.color_image = image_color
    algorithm.Step()