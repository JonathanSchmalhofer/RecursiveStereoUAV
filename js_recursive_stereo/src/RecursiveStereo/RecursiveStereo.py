import numpy as np
import cv2
from matplotlib import pyplot as plt

class RecursiveStereo:
    def __init__(self):
        self.left_image  = None
        self.right_image = None
        self.blockmatching_blocksize = 16
        self.blockmatching_window_size = 3
        self.blockmatching_minimum_disparities = 16
        self.blockmatching_number_disparities = 80-self.blockmatching_minimum_disparities
    
    def SetLeftImage(self, image):
        self.left_image = image
    
    def SetRightImage(self, image):
        self.right_image = image
    
    def RequirementsFulfilled(self):
        if self.left_image is None:
            return False
        if self.right_image is None:
            return False
        return True
    
    def GetDisparityImage(self):
            stereo = cv2.StereoSGBM_create(minDisparity = self.blockmatching_minimum_disparities,
                                   numDisparities = self.blockmatching_number_disparities,
                                   blockSize = self.blockmatching_blocksize,
                                   P1 = 8*3*self.blockmatching_window_size**2,
                                   P2 = 32*3*self.blockmatching_window_size**2,
                                   disp12MaxDiff = 1,
                                   uniquenessRatio = 10,
                                   speckleWindowSize = 100,
                                   speckleRange = 32
                                 )
            disparity = stereo.compute(self.left_image, self.right_image)
            return disparity
    
    def Step(self):
        if self.RequirementsFulfilled():
            print("Go")
            disparity = self.GetDisparityImage()
        else:
            print("Something is missing")