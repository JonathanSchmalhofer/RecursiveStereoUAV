#!/usr/bin/env python
import os
import cv2
from RecursiveStereoPackage.RecursiveStereo import RecursiveStereo

if __name__ == '__main__':
    algorithm = RecursiveStereo()
    algorithm.Step()
    image_left  = cv2.imread('../resources/left_0000000000.png')
    image_right = cv2.imread('../resources/right_0000000000.png')
    algorithm.left_image  = image_left
    algorithm.right_image = image_right
    algorithm.Step()