import numpy as np
import cv2

# Copied from https://github.com/utiasSTARS/pykitti/blob/master/pykitti/utils.py
def read_calib_file(filepath):
    """Read in a calibration file and parse into a dictionary."""
    data = {}
    with open(filepath, 'r') as f:
        for line in f.readlines():
            key, value = line.split(':', 1)
            # The only non-float values in these files are dates, which
            # we don't care about anyway
            try:
                data[key] = np.array([float(x) for x in value.split()])
            except ValueError:
                pass
    return data

calibration_data = read_calib_file('D:\\zWork\\01_Fernuni_Hagen\\01 - M.Sc. Elektro- und Informationstechnik\\98 - Masterarbeit\\04_KITTI_Benchmark\\Road\\2011_09_26_drive_0027\\2011_09_26\\calib_cam_to_cam.txt')
P_rect_cam_left  = calibration_data['P_rect_00'].reshape((3,4))
P_rect_cam_right = calibration_data['P_rect_01'].reshape((3,4))

# Focal Length in [pix]
f                           =    P_rect_cam_left[0,0]
# u-Coordinate of Center Point in [pix]                  
c_u                         =    P_rect_cam_left[0,2]
# u-Coordinate of Center Point in [pix]             
c_v                         =    P_rect_cam_left[1,2]
# baseline with respect to reference camera 0 in [m]
b                           = -1*P_rect_cam_right[0,3]/f

Q = np.matrix(
    [[ 0,         0,         0,          f],
     [-1.0000,    0,         0,          c_u],
     [ 0,        -1.0000,    0,          c_v],
     [ 0,         0,         1/b,        0]])

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

# Following part for converting a disparity image to a PointCloud was originally copied from
# https://stackoverflow.com/questions/45325795/point-cloud-from-kitti-stereo-images

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