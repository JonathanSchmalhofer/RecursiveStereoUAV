close all; dbstop error; clc;

disp( '======= PointCloud Generator from Disparity =======' );

load('..\..\resources\AirSimCameraCalibration\stereoCalibrationResults.mat')
uPatchSize  = 5;
vPatchSize  = 5;

disp( '########### Processign Frame ');
imgL        = rgb2gray(imread( '../../resources/AirSimNeighbourhood/images/left_00012.png' ));
imgR        = rgb2gray(imread( '../../resources/AirSimNeighbourhood/images/right_00012.png' ));
imgLC       = imread( '../../resources/AirSimNeighbourhood/images/left_00012.png' );
%imgL        = rgb2gray(imread( '../../resources/AirSimCameraCalibration/left/left_00000.png' ));
%imgR        = rgb2gray(imread( '../../resources/AirSimCameraCalibration/right/right_00000.png' ));
%imgLC       = imread( '../../resources/AirSimCameraCalibration/left/left_00000.png' );
[ height, width ] = size( imgL );


%[imgL,imgR] = rectifyStereoImages(imgL,imgR,stereoParams)

disparityMap = disparity(imgL, imgR, 'Method', 'SemiGlobal');

figure; imshowpair(imgL,imgR,'ColorChannels','red-cyan');
title('Red-cyan composite view of the stereo images');
% For the purpose of visualizing the disparity, replace the -realmax('single') marker with the minimum disparity value.
disparityMapVis = disparityMap;
marker_idx = (disparityMapVis == -realmax('single'));
disparityMapVis(marker_idx) = min(disparityMapVis(~marker_idx));
% Show the disparity map. Brighter pixels indicate objects which are closer to the camera.
figure; imshow(mat2gray(disparityMapVis));



f                           =    mean(stereoParams.CameraParameters1.FocalLength); % Focal Length in [pix]
c_u                         =    stereoParams.CameraParameters1.PrincipalPoint(1); % u-Coordinate of Center Point in [pix]
c_v                         =    stereoParams.CameraParameters1.PrincipalPoint(2); % v-Coordinate of Center Point in [pix]
% For baseline in AirSim, see: https://github.com/Microsoft/AirSim/issues/323
b                           =    0.14%0.2506011381/2%0.501202762;%250;%270.5466;       % baseline with respect to reference camera 0 in [m]

%Width  = size(imgL,2);
%Height = size(imgL,1);
%FOV    = 70*pi/180;
%f      = Width/ 2 / tan(FOV/2);
%c_u    = Width/2;
%c_v    = Height/2;
%b      = 0.14;



%
%
% Set up Transformation Matrix D (disparity-to-depth)
D           = eye(7,7);
D(1,1:4)    = [  0,    0,    0,    f   ];
D(2,1:4)    = [ -1,    0,    0,    c_u ];
D(3,1:4)    = [  0,   -1,    0,    c_v ];
D(4,1:4)    = [  0,    0,  1/b,    0   ];


data        = arrangeData( disparityMap, imgLC, uPatchSize, vPatchSize );
%
p          = (D*data')';
x          = p(:,1:3)./[ p(:,4), p(:,4), p(:,4) ];
colors     = p(:,5:7);
idx        = ~any( isnan( x ) | isinf( x ), 2 );
x          = x( idx, : );
colors     = colors( idx, : );

pose                    = { eye(4) };
x_cam                   = ( pose{ 1 } * [ x, ones( size( x, 1 ), 1 ) ]' )';
x_all{ 1 }        = x_cam(:,1:3)./[ x_cam(:,4), x_cam(:,4), x_cam(:,4) ];

colors_all{ 1 }   = colors;

mergedPointCloud = pointCloud( x_all{ 1 }, 'Color', uint8( colors_all{ 1 } ) );
pcwrite( mergedPointCloud, 'mergedPointCloud', 'PLYFormat', 'binary' );

figure; axis off;
showPointCloud(mergedPointCloud, 'MarkerSize', 100);
set(gca, 'CameraPosition', [0,0,0]);
set(gca, 'CameraTarget', [65,0,0]);