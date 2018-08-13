close all; dbstop error; clc;

disp( '======= PointCloud Generator from Velodyne Laser Point Cloud =======' );
%
calib_dir   = '';

%
% Configuration
camL        = 0;    % 0-based index
camR        = 1;    % 0-based index
frame       = 0;    % 0-based index
uPatchSize  = 5;
vPatchSize  = 5;

%
% load oxts data
ts          = loadTimestamps('./oxts');
oxts        = loadOxtsliteData('.');
%
% transform to poses
pose        = convertOxtsToPose(oxts);
%
% calculate delta times and delta poses
deltaPose 	= cell( size( pose ) );
timeStruct  = cell( size( pose ) );
timeStamp   = zeros( size( pose ) );
token       = '(?<year>[\d]+)\-(?<month>[\d]+)\-(?<day>[\d]+)\s(?<hour>[\d]+)\:(?<minute>[\d]+)\:(?<second>[\d]+\.[\d]+)';


%
% Load Calibration
calib                       = loadCalibrationCamToCam(  fullfile( calib_dir, 'calib_cam_to_cam.txt' ) );
Tr_imu_to_velo              = loadCalibrationRigid(     fullfile( calib_dir, 'calib_imu_to_velo.txt' ) );
Tr_velo_to_cam              = loadCalibrationRigid(     fullfile( calib_dir, 'calib_velo_to_cam.txt' ) );
% Extract Rectifying Rotation Matrix
R_cam_to_rect               =    eye(4);
R_cam_to_rect(1:3,1:3)      =    calib.R_rect{1};
% Extract Intrinsic Camera Parameter
P_rect                      =    calib.P_rect;
f                           =    P_rect{camL+1}(1,1);                  % Focal Length in [pix]
c_u                         =    P_rect{camL+1}(1,3);                  % u-Coordinate of Center Point in [pix]
c_v                         =    P_rect{camL+1}(2,3);                  % u-Coordinate of Center Point in [pix]
b                           = -1*P_rect{max([camL+1;2])}(1,4)/f;       % baseline with respect to reference camera 0 in [m]
%
%
% Set up Transformation Matrix D (disparity-to-depth)
D           = eye(7,7);
D(1,1:4)    = [  0,    0,    0,    f   ];
D(2,1:4)    = [ -1,    0,    0,    c_u ];
D(3,1:4)    = [  0,   -1,    0,    c_v ];
D(4,1:4)    = [  0,    0,  1/b,    0   ];
%
%
%
Tr_imu_to_cam               = R_cam_to_rect*Tr_velo_to_cam*Tr_imu_to_velo;
%
%
frame = 0;

% load velodyne points
fid = fopen('../../resources/velodyne_0000000000.bin','rb');
velo = fread(fid,[4 inf],'single')';
%velo = velo(1:5:end,:); % remove every 5th point for display speed
fclose(fid);

colors      = zeros(size(velo,1),3);
colors(:,1) = velo(:,4) * 255;

mergedPointCloud = pointCloud( velo(:,1:3), 'Color', uint8( colors ) );
pcwrite( mergedPointCloud, 'mergedPointCloud_velodyne', 'PLYFormat', 'binary' );