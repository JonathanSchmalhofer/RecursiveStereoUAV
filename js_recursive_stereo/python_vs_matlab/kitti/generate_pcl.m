close all; dbstop error; clc;

disp( '======= PointCloud Generator from Disparity =======' );
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
disp( [ '########### Processign Frame #', num2str( frame ) ] );
imgL        = imread( 'left_0000000000.png' );
imgR        = imread( 'right_0000000000.png' );
imgLC       = imread( 'color_0000000000.png' );
[ height, width ] = size( imgL );
%disparityMap = disparity(left, right, 'Method', 'BlockMatching','BlockSize',7);
disparityMap = disparity(imgL, imgR, 'Method', 'SemiGlobal');
    
data        = arrangeData( disparityMap, imgLC, uPatchSize, vPatchSize );
%
p           = (D*data')';
x           = p(:,1:3)./[ p(:,4), p(:,4), p(:,4) ];
colors      = p(:,5:7);
selVec      = x(:,1);
%idx         = find(selVec < 80);
%x           = x(idx,:);
%colors      = colors(idx,:);
selVec      = x(:,2);
%idx         = find(abs(selVec) < 30);
%x           = x(idx,:);
%colors      = colors(idx,:);
selVec      = x(:,3);
%idx         = find(selVec < 30);
%x           = x(idx,:);
%colors      = colors(idx,:);
selVec      = x(:,3);
%idx         = find(selVec > -30);
%x           = x(idx,:);
%colors      = colors(idx,:);

x_imu                   = (pose{ frame+1 } * ( inv(Tr_imu_to_cam) * [ x, ones( size( x, 1 ), 1 ) ]' ) )';
x_imu                   = x_imu(:,1:3)./[ x_imu(:,4), x_imu(:,4), x_imu(:,4) ];
x_cam                   = ( pose{ frame+1 } * [ x, ones( size( x, 1 ), 1 ) ]' )';
x_all{ frame+1 }        = x_cam(:,1:3)./[ x_cam(:,4), x_cam(:,4), x_cam(:,4) ];
x_imu_all{ frame+1 }    = x_imu;
colors_all{ frame+1 }   = colors;


%
%     figure();
%     for iFrame = 1:numel( x_all )
%         currX       = x_all{ iFrame };
%         currCol     = colors_all{ iFrame };
%         for iPoint = 1:size( currX, 1 )
%             plot3( currX(iPoint,1), currX(iPoint,2), currX(iPoint,3), '.', 'Color', currCol( iPoint, : )./255  );
%             hold on;
%         end
%     end
%     hold off;
%     axis equal;
%
%     figure;
%     for iFrame = 1:numel( x_imu_all )
%         currX       = x_imu_all{ iFrame };
%         currCol     = colors_all{ iFrame };
%         for iPoint = 1:size( currX, 1 )
%             plot3( currX(iPoint,1), currX(iPoint,2), currX(iPoint,3), '.', 'Color', currCol( iPoint, : )./255  );
%             %plot3( currX(iPoint,1), currX(iPoint,2), currX(iPoint,3), '.', 'Color', [255 0 0]./255  );
%             hold on;
%         end
%     end
%     axis equal;
%     hold off;
%
mergedPointCloud = pointCloud( x_all{ 1 }, 'Color', uint8( colors_all{ 1 } ) );
pcwrite( mergedPointCloud, 'mergedPointCloud', 'PLYFormat', 'binary' );