import numpy as np
import cv2
from matplotlib import pyplot as plt

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

class RecursiveStereo:
    def __init__(self):
        # Attributes
        self.left_image  = None
        self.right_image = None
        self.pcl         = None
        self.disparity   = None
        
        # Configuration
        self.export_pcl       = False
        self.enable_recursive = True
        
        # Parameters for PCL Generation
        self.c_u         = None # default from KITTI: 609.5593
        self.c_v         = None # default from KITTI: 172.8540
        self.b           = None # default from KITTI: 0.5372
        self.f           = None # default from KITTI: 721.5377
        
        # Parameters for (SemiGlobal)BlockMatching
        self.blockmatching_blocksize           = None # default used for KITTI: 15
        self.blockmatching_window_size         = None # default used for KITTI: 9
        self.blockmatching_minimum_disparities = None # default used for KITTI: -64
        self.blockmatching_maximum_disparities = None # default used for KITTI: 128
    
    def RequirementsFulfilled(self):
        if self.left_image is None:
            print('No left image set')
            return False
        if self.right_image is None:
            print('No right image set')
            return False
        if self.c_u is None:
            print('No c_u parameter set')
            return False
        if self.c_v is None:
            print('No c_v parameter set')
            return False
        if self.b is None:
            print('No b parameter set')
            return False
        if self.b == 0:
            print('Parameter b may not be zero')
            return False
        if self.f is None:
            print('No f parameter set')
            return False
        if self.blockmatching_blocksize is None:
            print('No blocksize parameter for blockmatching set')
            return False
        if self.blockmatching_window_size is None:
            print('No window size parameter for blockmatching set')
            return False
        if self.blockmatching_minimum_disparities is None:
            print('No minimum disparity parameter for blockmatching set')
            return False
        if self.blockmatching_maximum_disparities is None:
            print('No maximum disparity parameter for blockmatching set')
            return False
        self.PrintParameters()
        return True
    
    def PrintParameters(self):
        print("    Running RecursiveStereo with following parameters:")
        print(" ")
        print("        # Configuration")
        print("        self.export_pcl       = {}".format(self.export_pcl))
        print("        self.enable_recursive = {}".format(self.enable_recursive))
        print(" ")
        print("        # Parameters for PCL Generation")
        print("        self.c_u         = {}".format(self.c_u))
        print("        self.c_v         = {}".format(self.c_v))
        print("        self.b           = {}".format(self.b))
        print("        self.f           = {}".format(self.f))
        print(" ")
        print("        # Parameters for (SemiGlobal)BlockMatching")
        print("        self.blockmatching_blocksize           = {}".format(self.blockmatching_blocksize))
        print("        self.blockmatching_window_size         = {}".format(self.blockmatching_window_size))
        print("        self.blockmatching_minimum_disparities = {}".format(self.blockmatching_minimum_disparities))
        print("        self.blockmatching_maximum_disparities = {}".format(self.blockmatching_maximum_disparities))
    
    def GeneratePCLFromDisparity(self, disparity):
        Q = np.float32([[ 1, 0,  0,        -self.c_u ],
                        [ 0, 1,  0,        -self.c_v ],
                        [ 0, 0,  0,         self.f   ],
                        [ 0, 0, -1/self.b,  0        ]])
        pcl = cv2.reprojectImageTo3D(disparity, Q)
        return pcl
    
    def GetDisparityImage(self):
            stereo = cv2.StereoSGBM_create(minDisparity = self.blockmatching_minimum_disparities,
                                           numDisparities = (self.blockmatching_maximum_disparities-self.blockmatching_minimum_disparities),
                                           blockSize = self.blockmatching_blocksize,
                                           P1 = 4*3*self.blockmatching_window_size**2,
                                           P2 = 32*3*self.blockmatching_window_size**2,
                                           preFilterCap = 10
                                          )
            disparity = stereo.compute(self.left_image, self.right_image).astype(np.float32) / 16.0
            return disparity
    
    # Copied from https://github.com/opencv/opencv/blob/master/samples/python/stereo_match.py
    def ExportPCLToPly(self, filename, vertices):
        vertices = vertices.reshape(-1, 3)
        with open(filename, 'wb') as file:
            file.write((ply_header % dict(vert_num=len(vertices))).encode('utf-8'))
            np.savetxt(file, vertices, fmt='%f %f %f 0 0 0 ')
    
    def Step(self):
        if self.RequirementsFulfilled():
            print("Step")
            # Get disparity image
            disparity = self.GetDisparityImage()
            self.disparity = disparity
            # Get point cloud
            points = self.GeneratePCLFromDisparity(disparity)
            mask = ((disparity < disparity.max()) &
                    (disparity > disparity.min()) &
                    (np.isfinite(disparity)))
            pcl = points[mask]
            # Export point cloud
            if self.export_pcl == True:
                pcl_filename = 'out.ply'
                self.ExportPCLToPly(pcl_filename, pcl)
            self.pcl = pcl
            if self.enable_recursive == True:
                pass
        else:
            print("Not all requirements fulfilled for calculation step")