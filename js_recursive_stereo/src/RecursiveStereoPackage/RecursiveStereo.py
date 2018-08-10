import numpy as np
import cv2
from matplotlib import pyplot as plt

class RecursiveStereo:
    def __init__(self):
        # Attributes
        self.left_image  = None
        self.right_image = None
        self.pcl         = None
        self.c_u         = 609.5593
        self.c_v         = 172.8540
        self.b           = 0.5372
        self.f           = 721.5377
        self.disparity   = None
        
        # Parameters for (SemiGlobal)BlockMatching
        self.blockmatching_blocksize = 15
        self.blockmatching_window_size = 5
        self.blockmatching_minimum_disparities = -64
        self.blockmatching_maximum_disparities = 128
    
    def RequirementsFulfilled(self):
        if self.left_image is None:
            return False
        if self.right_image is None:
            return False
        return True
        
    def GeneratePCLFromDisparity(self, disparity):
        Q = np.float32([[ 1, 0,  0,        -self.c_u ],
                        [ 0, 1,  0,        -self.c_v ],
                        [ 0, 0,  0,         self.f   ],
                        [ 0, 0, -1/self.b,  0        ]])
        points = cv2.reprojectImageTo3D(disparity, Q)
        return points
    
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
            disparity = stereo.compute(self.left_image, self.right_image).astype(np.float32) / 16.0
            return disparity
    
    # Copied from https://github.com/opencv/opencv/blob/master/samples/python/stereo_match.py
    def ExportPCLToPly(self, filename, vertices):
        ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''
        vertices = vertices.reshape(-1, 3)
        with open(filename, 'wb') as file:
            file.write((ply_header % dict(vert_num=len(vertices))).encode('utf-8'))
            np.savetxt(file, vertices, fmt='%f %f %f 0 0 0 ')
    
    def Step(self):
        if self.RequirementsFulfilled():
            print("Go")
            disparity = self.GetDisparityImage()
            self.disparity = disparity
            points = self.GeneratePCLFromDisparity(disparity)
            self.pcl = points
            mask = ((disparity < disparity.max()) & (disparity > disparity.min()))
            out_points = points[mask]
            out_fn = 'out.ply'
            self.ExportPCLToPly('out.ply', out_points)
            print('%s saved' % 'out.ply')
        else:
            print("Something is missing")