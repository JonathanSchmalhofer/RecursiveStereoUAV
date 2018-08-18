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
        self.verbose     = True
        self.pose        = self.EmptyPose()
        self.last_pose   = self.EmptyPose()
        self.color_image = None
        self.left_image  = None
        self.right_image = None
        self.pcl         = None
        self.pcl_reduced = None
        self.disparity   = None
        self.orb         = cv2.ORB_create(nfeatures=50, scoreType=cv2.ORB_FAST_SCORE) # Initiate ORB  detector
        self.sift        = cv2.xfeatures2d.SIFT_create(nfeatures=50)                  # Initiate SIFT detector
        
        # Configuration
        self.export_pcl       = False
        self.enable_recursive = True
        self.pcl_filename     = 'out.ply'
        
        # Grid Coordinates
        self.u_step_px = 10
        self.v_step_px = 10
        
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
    
    def EmptyPose(self):
        pose = {'position':    { 'x': 0, 'y': 0, 'z': 0 },
                'orientation': { 'x': 0, 'y': 0, 'z': 0, 'w': 1 }}
        return pose
    
    def CreateWallBehindCamera(self, min_distance = 1):
        pcl_wall = []
        brick_size = 0.2
        for x in np.arange(-1,+0.6,brick_size):
            for y in np.arange(-1.4,+1.4,brick_size):
                for z in np.arange(-1.5,+1.5,brick_size):
                    dist_to_cam = np.linalg.norm(np.array([x,y,z]) - np.array([0,0,0]))
                    if dist_to_cam >= min_distance:
                        new_point = np.array([x,y,z,1,0,0,0]) # format [x,y,z,1,r,g,b]
                        pcl_wall.append(new_point) 
        return np.array(pcl_wall)
                    
    
    def GetSiftKeyPoints(self, image):
        # find the keypoints and compute the descriptors with SIFT
        kp, des = self.sift.detectAndCompute(left_image, None)
        grid    = np.int16( [(kpoint.pt[1], kpoint.pt[0]) for kpoint in kp ] )
        u_idx   = [kpoint.pt[1] for kpoint in kp_orb]
        v_idx   = [kpoint.pt[0] for kpoint in kp_orb]
        return grid, u_idx, v_idx
    
    def GetOrbKeyPoints(self, image):
        # find the keypoints with ORB
        kp      = self.orb.detect(image, None)
        # compute the descriptors with ORB
        kp, des = self.orb.compute(image, kp)
        grid    = np.int16( [(kpoint.pt[1], kpoint.pt[0]) for kpoint in kp ] )
        return grid
    
    def Get2dGridCoordinates(self):
        width  = self.left_image.shape[1]
        height = self.left_image.shape[0]
        u_list = np.arange(self.u_step_px, width,  self.u_step_px, dtype=np.int16)
        v_list = np.arange(self.v_step_px, height, self.v_step_px, dtype=np.int16)
        grid   = [(v, u) for u in u_list for v in v_list ]
        return grid
    
    def Initialize2dCoordinates(self):
        grid      = self.Get2dGridCoordinates()
        grid_orb  = self.GetOrbKeyPoints(self.left_image)
        #grid_sift = self.GetSiftKeyPoints(self.left_image)
        grid.extend(grid_orb)
        grid = np.int16(grid)
        return grid
    
    def GetQ(self):
        Q    = np.float32(
               [[ 0,         0,         0,          self.f  ],
                [-1.0000,    0,         0,          self.c_u],
                [ 0,        -1.0000,    0,          self.c_v],
                [ 0,         0,         1/self.b,   0       ]])
        return Q
    
    # Copied from: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    def QuaternionToRollPitchYaw(self, q):
        t0 = +2.0 * (q['w'] * q['x'] + q['y'] * q['z'])
        t1 = +1.0 - 2.0 * (q['x'] * q['x'] + q['y'] * q['y'])
        roll = np.arctan2(t0, t1)
        
        t2 = +2.0 * (q['w'] * q['y'] - q['z'] * q['x'])
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)
        
        t3 = +2.0 * (q['w'] * q['z'] + q['x'] * q['y'])
        t4 = +1.0 - 2.0 * (q['y'] * q['y'] + q['z'] * q['z'])
        yaw = np.arctan2(t3, t4)
        return roll, pitch, yaw
    
    def GetPoseMatrix(self):
        # init as identity
        Rx = np.identity(4)
        Rx = np.identity(4)
        Ry = np.identity(4)
        Rz = np.identity(4)
        Tr = np.identity(4)
        
        # Get current orientation in r-p-y
        roll, pitch, yaw = self.QuaternionToRollPitchYaw(self.pose['orientation'])
        
        Rx[0,0] = 1;                Rx[0,1] = 0;            Rx[0,2] = 0
        Rx[1,0] = 0;                Rx[1,1] = np.cos(roll); Rx[1,2] = -np.sin(roll)
        Rx[2,0] = 0;                Rx[2,1] = np.sin(roll); Rx[2,2] =  np.cos(roll)
        
        Ry[0,0] =  np.cos(pitch);   Ry[0,1] = 0;            Ry[0,2] = np.sin(pitch)
        Ry[1,0] =  0;               Ry[1,1] = 1;            Ry[1,2] = 0
        Ry[2,0] = -np.sin(pitch);   Ry[2,1] = 0;            Ry[2,2] = np.cos(pitch)
        
        Rz[0,0] = np.cos(yaw);      Rz[0,1] = -np.sin(yaw); Rz[0,2] = 0
        Rz[1,0] = np.sin(yaw);      Rz[1,1] =  np.cos(yaw); Rz[1,2] = 0
        Rz[2,0] = 0;                Rz[2,1] =  0;           Rz[2,2] = 1
        
        # R = Rz*(Ry*Rx)
        R = np.dot(Ry, Rx)
        R = np.dot(Rz, R)
        
        # in Matlab notation: Tr = [R t;0 0 0 1];
        Tr[0,0] = R[0,0]; Tr[0,1] = R[0,1]; Tr[0,2] = R[0,2]; Tr[0,3] = self.pose['position']['x']
        Tr[1,0] = R[1,0]; Tr[1,1] = R[1,1]; Tr[1,2] = R[1,2]; Tr[1,3] = self.pose['position']['y']
        Tr[2,0] = R[2,0]; Tr[2,1] = R[2,1]; Tr[2,2] = R[2,2]; Tr[2,3] = self.pose['position']['z']
        Tr[3,0] = 0;      Tr[3,1] = 0;      Tr[3,2] = 0;      Tr[3,3] = 1;
        return Tr
    
    def GetPoseColorMatrix(self):
        D = np.float32(np.identity(7))
        T = self.GetPoseMatrix()
        D[0:4,0:4] = T # set upper left 4x4 matrix to T, rest to identity (for color)
        return D
    
    def GetPCLFromDisparity(self, disparity):
        Q    = self.GetQ()
        pcl  = cv2.reprojectImageTo3D(disparity, Q) # returns shape (height,width,3)
        
        ## NOT NEEDED, AS APPLYING THE MASK WILL ALLREADY RESHAPE
        # Reshape from (height,width,3) to (height*width,3)
        #pcl = np.array(pcl).reshape(-1,1,3).squeeze()
        
        # Filter out invalid points
        mask = ((disparity < disparity.max()) &
                (disparity > disparity.min()) &
                (np.isfinite(pcl[:,:,0]))          &
                (np.isfinite(pcl[:,:,1]))          &
                (np.isfinite(pcl[:,:,2])))
        pcl = pcl[mask] # returns shape ((height*width)-x,3) with x being the number of points filtered out by mask
        
        # If no color image was given, make all points black
        if self.color_image is None:
            pcl_c = np.append(pcl.transpose(),[np.zeros(pcl.shape[0])],axis=0).transpose() # format: [x, y, z, r, g, b] with r = g = b = 0 (black)
        else:
            pcl_c = np.append(pcl,self.color_image[mask],axis=1) # format: [x, y, z, r, g, b]
        return pcl_c
    
    def GetReducedPCLFromDisparity(self, disparity):
        # Get Sampling Points
        grid              = self.Initialize2dCoordinates()
        u_idx             = [x[1]-1 for x in grid]
        v_idx             = [x[0]-1 for x in grid]
        disparity_reduced = disparity[v_idx,u_idx]
        
        Q                 = self.GetQ()
        y                 = np.float32([[x[1], x[0], disparity[x[0]-1,x[1]-1], 1] for x in grid ]) # [u,v,disp(v,u),1]
        pcl               = np.dot(Q, np.transpose(y)).transpose()
        pcl_norm          = np.array([x/x[3] for x in pcl]) # normalize
        # Due to the different dimensions, the reduced mask is slight differntly setup
        mask = ((disparity_reduced < disparity_reduced.max()) &
                (disparity_reduced > disparity_reduced.min()) &
                (np.isfinite(pcl_norm[:,0]))               &
                (np.isfinite(pcl_norm[:,1]))               &
                (np.isfinite(pcl_norm[:,2])))
        pcl_reduced       = pcl_norm[mask]
        
        # If no color image was given, make all points black
        if self.color_image is None:
            pcl_reduced_c = np.append(pcl_reduced.transpose(),[np.zeros(pcl_reduced.shape[0])],axis=0).transpose() # format: [x, y, z, r, g, b] with r = g = b = 0 (black)
        else:
            color_reduced = np.array([self.color_image[x[0]-1,x[1]-1] for x in grid ])
            pcl_reduced_c = np.append(pcl_reduced,color_reduced[mask],axis=1) # format: [x, y, z, r, g, b]
        return pcl_reduced_c
    
    def TransformPCL(self, pcl):
        # make affine coordinates: [[x,y,z,r,g,b],...] to [[x,y,z,1,r,g,b],...]
        pcl_aff = np.vstack((pcl[:,0],
                             pcl[:,1],
                             pcl[:,2],
                             [np.ones(pcl.shape[0])],
                             pcl[:,3],
                             pcl[:,4],
                             pcl[:,5]))
        D = self.GetPoseColorMatrix()
        
        pcl_transf = np.dot(D,pcl_aff).transpose()
        pcl_transf = np.array([[x[0]/x[3],
                                x[1]/x[3],
                                x[2]/x[3],
                                x[4],
                                x[5],
                                x[6]] for x in pcl_transf]) # normalize and back to format [[x,y,z,r,g,b],...]
        return pcl_transf
    
    def AppendPCL(self, current, to_append):
        if current is None:
            current = to_append
        else:
            current = np.append(current,to_append,axis=0)
        return current
    
    def GetDisparityImage(self):
            stereo = cv2.StereoSGBM_create(minDisparity   = self.blockmatching_minimum_disparities,
                                           numDisparities = (self.blockmatching_maximum_disparities-self.blockmatching_minimum_disparities),
                                           blockSize      = self.blockmatching_blocksize,
                                           P1             = 4*3*self.blockmatching_window_size**2,
                                           P2             = 32*3*self.blockmatching_window_size**2,
                                           preFilterCap   = 10
                                          )
            disparity = stereo.compute(self.left_image, self.right_image).astype(np.float32) / 16.0
            return disparity
    
    # Copied from https://github.com/opencv/opencv/blob/master/samples/python/stereo_match.py
    def ExportPCLToPly(self, filename, vertices_with_color):
        vertices_with_color = vertices_with_color.reshape(-1, 6)
        with open(filename, 'wb') as file:
            file.write((ply_header % dict(vert_num=len(vertices_with_color))).encode('utf-8'))
            np.savetxt(file, vertices_with_color, fmt='%f %f %f %d %d %d ')
    
    def Step(self):
        if self.RequirementsFulfilled():
            #self.PrintParameters()
            
            ## F U L L
            # Get disparity image
            self.disparity = self.GetDisparityImage()
            
            # Get full point cloud - in camera coordinate system
            #pcl_cam = self.GetPCLFromDisparity(self.disparity)
            
            # Transform to inertial coordinate system
            #pcl_inert = self.TransformPCL(pcl_cam)
            
            # Save
            #self.pcl = self.AppendPCL(self.pcl,pcl_inert)
            
            # Export
            #if self.export_pcl == True:
            #    self.ExportPCLToPly(self.pcl_filename, self.pcl)
            
            ## R E D U C E D
            # Get reduced point cloud - in camera coordinate system
            pcl_reduced_cam = self.GetReducedPCLFromDisparity(self.disparity)
            pcl_wall        = self.CreateWallBehindCamera()

            pcl_reduced_cam = self.AppendPCL(pcl_reduced_cam, pcl_wall)
            
            # Transform to inertial coordinate system
            pcl_reduced_inert = self.TransformPCL(pcl_reduced_cam)
            
            # Save
            self.pcl = self.AppendPCL(self.pcl, pcl_reduced_inert)
            
            # Export
            #if self.export_pcl == True:
            #    self.ExportPCLToPly('reduced.ply', self.pcl_reduced)
            
            if self.enable_recursive == True:
                pass
        else:
            self.VerbosePrint("Not all requirements fulfilled for calculation step")

################################################################################
# Only convenience functions below

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
    