import numpy as np
import cv2

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

def write_ply(fn, verts, colors):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')

imgC = cv2.imread('color_0000000000.png')
imgL = cv2.imread('left_0000000000.png')
imgR = cv2.imread('right_0000000000.png')

window_size = 9
minDisparity = 1
stereo = cv2.StereoSGBM_create(
    blockSize=10,
    numDisparities=64,
    preFilterCap=10,
    minDisparity=minDisparity,
    P1=4 * 3 * window_size ** 2,
    P2=32 * 3 * window_size ** 2
)

# K_xx: 3x3 calibration matrix of camera xx before rectification
K_L = np.matrix(
    [[9.597910e+02, 0.000000e+00, 6.960217e+02],
     [0.000000e+00, 9.569251e+02, 2.241806e+02],
     [0.000000e+00, 0.000000e+00, 1.000000e+00]])
K_R = np.matrix(
    [[9.037596e+02, 0.000000e+00, 6.957519e+02],
     [0.000000e+00, 9.019653e+02, 2.242509e+02],
     [0.000000e+00, 0.000000e+00, 1.000000e+00]])

# D_xx: 1x5 distortion vector of camera xx before rectification
D_L = np.matrix([-3.691481e-01, 1.968681e-01, 1.353473e-03, 5.677587e-04, -6.770705e-02])
D_R = np.matrix([-3.639558e-01, 1.788651e-01, 6.029694e-04, -3.922424e-04, -5.382460e-02])

# R_xx: 3x3 rotation matrix of camera xx (extrinsic)
R_L = np.transpose(np.matrix([[9.999758e-01, -5.267463e-03, -4.552439e-03],
                              [5.251945e-03, 9.999804e-01, -3.413835e-03],
                              [4.570332e-03, 3.389843e-03, 9.999838e-01]]))
R_R = np.matrix([[9.995599e-01, 1.699522e-02, -2.431313e-02],
                 [-1.704422e-02, 9.998531e-01, -1.809756e-03],
                 [2.427880e-02, 2.223358e-03, 9.997028e-01]])

# T_xx: 3x1 translation vector of camera xx (extrinsic)
T_L = np.transpose(np.matrix([5.956621e-02, 2.900141e-04, 2.577209e-03]))
T_R = np.transpose(np.matrix([-4.731050e-01, 5.551470e-03, -5.250882e-03]))

IMG_SIZE = (1392, 512)

rotation = R_L * R_R
translation = T_L - T_R

# output matrices from stereoRectify init
R1 = np.zeros(shape=(3, 3))
R2 = np.zeros(shape=(3, 3))
P1 = np.zeros(shape=(3, 4))
P2 = np.zeros(shape=(3, 4))
Q = np.zeros(shape=(4, 4))

R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(K_L, D_L, K_R, D_R, IMG_SIZE, rotation, translation,
                                                                  R1, R2, P1, P2, Q,
                                                                  newImageSize=(1242, 375))

print('computing disparity...')
disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0


points = cv2.reprojectImageTo3D(disp, Q)
colors = cv2.cvtColor(imgC, cv2.COLOR_BGR2RGB)
mask = disp > disp.min()
out_points = points[mask]
out_colors = colors[mask]
out_fn = 'out.ply'
write_ply('out.ply', out_points, out_colors)
print('%s saved' % 'out.ply')