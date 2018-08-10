import numpy as np
import cv2
from matplotlib import pyplot as plt

class RecursiveStereo:
    def __init__(self):
        # Attributes
        self.left_image  = None
        self.right_image = None
        self.pcl         = None
        
        # Parameters for (SemiGlobal)BlockMatching
        self.blockmatching_blocksize = 15
        self.blockmatching_window_size = 3
        self.blockmatching_minimum_disparities = 0
        self.blockmatching_maximum_disparities = 64
    
    def RequirementsFulfilled(self):
        if self.left_image is None:
            return False
        if self.right_image is None:
            return False
        return True
        
    def GenerateDepthPCL(self, depth):
        """Transform a depth image into a point cloud with one point for each
        pixel in the image, using the camera transform for a camera
        centred at cx, cy with field of view fx, fy.

        depth is a 2-D ndarray with shape (rows, cols) containing
        depths from 1 to 254 inclusive. The result is a 3-D array with
        shape (rows, cols, 3). Pixels with invalid depth in the input have
        NaN for the z-coordinate in the result.
        
        Copied from: https://codereview.stackexchange.com/questions/79032/generating-a-3d-point-cloud
        """
        rows, cols = depth.shape
        c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
        valid = (depth > 0) & (depth < 255)
        z = np.where(valid, depth / 256.0, np.nan)
        x = np.where(valid, z * (c - self.cx) / self.fx, 0)
        y = np.where(valid, z * (r - self.cy) / self.fy, 0)
        return np.dstack((x, y, z))
    
    def GetDisparityImage(self):
            stereo = cv2.StereoSGBM_create(minDisparity = self.blockmatching_minimum_disparities,
                                           numDisparities = (self.blockmatching_maximum_disparities-self.blockmatching_minimum_disparities),
                                           blockSize = self.blockmatching_blocksize,
                                           P1 = 8*3*self.blockmatching_window_size**2,
                                           P2 = 32*3*self.blockmatching_window_size**2,
                                           disp12MaxDiff = 1,
                                           uniquenessRatio = 15,
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