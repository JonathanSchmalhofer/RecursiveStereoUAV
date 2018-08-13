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
        self.verbose     = False
        self.color_image = None
        self.left_image  = None
        self.right_image = None
        self.pcl         = None
        self.disparity   = None
        self.orb         = cv2.ORB_create() # Initiate ORB detector
        
        # Configuration
        self.export_pcl       = False
        self.enable_recursive = True
        self.pcl_filename     = 'out.ply'
        
        # Grid Coordinates
        self.u_step_px = 50
        self.v_step_px = 50
        
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
    
    def VerbosePrint(self, text):
        if self.verbose == True:
            print(text)
    
    def GetOrbKeyPoints(self, image):
        # find the keypoints with ORB
        kp      = self.orb.detect(image, None)
        # compute the descriptors with ORB
        kp, des = self.orb.compute(image, kp)
        #seems wrong --> TODO: u_list  = [kpoint.pt[1] for kpoint in kp]
        #seems wrong --> TODO: v_list  = [kpoint.pt[0] for kpoint in kp]
        #seems wrong --> TODO: grid    = np.int16( [(v, u) for u in u_list for v in v_list ] )
        grid    = np.int16( [(kpoint.pt[1], kpoint.pt[0]) for kpoint in kp ] )
        return grid#, u_list, v_list

    
    def Get2dGridCoordinates(self):
        width  = self.left_image.shape[1]
        height = self.left_image.shape[0]
        u_list = np.arange(self.u_step_px, width,  self.u_step_px, dtype=np.int16)
        v_list = np.arange(self.v_step_px, height, self.v_step_px, dtype=np.int16)
        grid   = [(v, u) for u in u_list for v in v_list ]
        return grid#, u_list, v_list
    
    def Initialize2dCoordinates(self):
        #grid, u_list, v_list             = self.Get2dGridCoordinates()
        #grid_orb, u_list_orb, v_list_orb = self.GetOrbKeyPoints(self.left_image)
        grid     = self.Get2dGridCoordinates()
        grid_orb = self.GetOrbKeyPoints(self.left_image)
        grid.extend(grid_orb)
        #u_list = np.append(u_list, u_list_orb)
        #v_list = np.append(v_list, v_list_orb)
        return grid#, u_list, v_list
    
    def RequirementsFulfilled(self):
        requirements_fulfilled = True
        if self.left_image is None:
            self.VerbosePrint('No left image set')
            requirements_fulfilled = False
        if self.right_image is None:
            self.VerbosePrint('No right image set')
            requirements_fulfilled = False
        if self.c_u is None:
            self.VerbosePrint('No c_u parameter set')
            requirements_fulfilled = False
        if self.c_v is None:
            self.VerbosePrint('No c_v parameter set')
            requirements_fulfilled = False
        if self.b is None:
            self.VerbosePrint('No b parameter set')
            requirements_fulfilled = False
        if self.b == 0:
            self.VerbosePrint('Parameter b may not be zero')
            requirements_fulfilled = False
        if self.f is None:
            self.VerbosePrint('No f parameter set')
            requirements_fulfilled = False
        if self.blockmatching_blocksize is None:
            self.VerbosePrint('No blocksize parameter for blockmatching set')
            requirements_fulfilled = False
        if self.blockmatching_window_size is None:
            self.VerbosePrint('No window size parameter for blockmatching set')
            requirements_fulfilled = False
        if self.blockmatching_minimum_disparities is None:
            self.VerbosePrint('No minimum disparity parameter for blockmatching set')
            requirements_fulfilled = False
        if self.blockmatching_maximum_disparities is None:
            self.VerbosePrint('No maximum disparity parameter for blockmatching set')
            requirements_fulfilled = False
        return requirements_fulfilled
    
    def PrintParameters(self):
        self.VerbosePrint("    Running RecursiveStereo with following parameters:")
        self.VerbosePrint(" ")
        self.VerbosePrint("        # Configuration")
        self.VerbosePrint("        self.export_pcl       = {}".format(self.export_pcl))
        self.VerbosePrint("        self.enable_recursive = {}".format(self.enable_recursive))
        self.VerbosePrint(" ")
        self.VerbosePrint("        # Parameters for PCL Generation")
        self.VerbosePrint("        self.c_u         = {}".format(self.c_u))
        self.VerbosePrint("        self.c_v         = {}".format(self.c_v))
        self.VerbosePrint("        self.b           = {}".format(self.b))
        self.VerbosePrint("        self.f           = {}".format(self.f))
        self.VerbosePrint(" ")
        self.VerbosePrint("        # Parameters for (SemiGlobal)BlockMatching")
        self.VerbosePrint("        self.blockmatching_blocksize           = {}".format(self.blockmatching_blocksize))
        self.VerbosePrint("        self.blockmatching_window_size         = {}".format(self.blockmatching_window_size))
        self.VerbosePrint("        self.blockmatching_minimum_disparities = {}".format(self.blockmatching_minimum_disparities))
        self.VerbosePrint("        self.blockmatching_maximum_disparities = {}".format(self.blockmatching_maximum_disparities))
    
    def GeneratePCLFromDisparity(self, disparity):
        Q = np.float32(
            [[ 0,         0,         0,          self.f],
             [-1.0000,    0,         0,          self.c_u],
             [ 0,        -1.0000,    0,          self.c_v],
             [ 0,         0,         1/self.b,   0]])
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
    def ExportPCLToPly(self, filename, vertices, colors = None):
        vertices = vertices.reshape(-1, 3)
        if colors is not None:
            colors = colors.reshape(-1, 3)
            vertices = np.hstack([vertices, colors])
        with open(filename, 'wb') as file:
            file.write((ply_header % dict(vert_num=len(vertices))).encode('utf-8'))
            if colors is None:
                np.savetxt(file, vertices, fmt='%f %f %f 0 0 0 ')
            else:
                np.savetxt(file, vertices, fmt='%f %f %f %d %d %d ')
    
    def Step(self):
        if self.RequirementsFulfilled():
            self.PrintParameters()
            # Get disparity image
            self.disparity = self.GetDisparityImage()
            print('self.disparity.shape = {}'.format(self.disparity.shape))
            # Get point cloud
            #grid, u_list, v_list = self.Initialize2dCoordinates()
            grid = self.Initialize2dCoordinates()
            u_idx  = [x[1] for x in grid]
            v_idx  = [x[0] for x in grid]
            print('len(grid) = {}'.format(len(grid)))
            #print('len(u_list) = {}'.format(len(u_list)))
            #print('len(v_list) = {}'.format(len(v_list)))
            print('len(u_idx) = {}'.format(len(u_idx)))
            print('len(v_idx) = {}'.format(len(v_idx)))
            #self.disparity = self.disparity[v_idx,u_idx].reshape(len(v_list),len(u_list))
            self.disparity = self.disparity[v_idx,u_idx]
            points = self.GeneratePCLFromDisparity(self.disparity)
            print('points.shape = {}'.format(points.shape))
            mask = ((self.disparity < self.disparity.max()) &
                    (self.disparity > self.disparity.min())
                    (np.isfinite(points[:,:,0]))            &
                    (np.isfinite(points[:,:,1]))            &
                    (np.isfinite(points[:,:,2])))
            print('len(points) = {}'.format(len(points)))
            print('len(mask) = {}'.format(len(mask)))
            print('points.shape = {}'.format(points.shape))
            print('mask.shape = {}'.format(mask.shape))
            print('self.disparity.shape = {}'.format(self.disparity.shape))
            print(mask)
            self.pcl = points[mask]
            self.export_pcl = False # Temporary
            # Export point cloud
            if self.export_pcl == True:
                if self.color_image is None:
                    self.ExportPCLToPly(self.pcl_filename, self.pcl)
                else:
                    self.ExportPCLToPly(self.pcl_filename, self.pcl, self.color_image[mask])
            if self.enable_recursive == True:
                pass
        else:
            self.VerbosePrint("Not all requirements fulfilled for calculation step")