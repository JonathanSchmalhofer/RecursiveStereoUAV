close all; dbstop error; clc;

disp( '======= Generate Figure for Presentation =======' );

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

figure; subplot(1,2,1);imshowpair(imgL,imgR);
% For the purpose of visualizing the disparity, replace the -realmax('single') marker with the minimum disparity value.
disparityMapVis = disparityMap;
marker_idx = (disparityMapVis == -realmax('single'));
disparityMapVis(marker_idx) = min(disparityMapVis(~marker_idx));
% Show the disparity map. Brighter pixels indicate objects which are closer to the camera.
subplot(1,2,2);imshow(mat2gray(disparityMapVis)); colormap jet; colorbar
axis off
set(gcf, 'Position', [ 10, 10, 1242, 375 ]);