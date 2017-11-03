%{
	eclect scripts to sanity check openCV implementation
	/Applications/MATLAB_R2017b.app/bin/matlab -nodesktop -nosplash

%}

clc;
close all;
clear all;

% % % % % % % % % % % % % % % % % % % % % % %
%  		ADD ROBOTICS PACKAGE PATH           %
% % % % % % % % % % % % % % % % % % % % % % %

kin_path1 = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/experiments/matlab/kinematics/kinematics/robotlinks';
kin_path2 = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/experiments/matlab/kinematics/kinematics/screws';
kin_path3 = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/experiments/matlab/kinematics/kinematics/utils';
kin_path4 = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/experiments/matlab/F_computation_v1';

addpath(kin_path1, kin_path2, kin_path3, kin_path4);

% % % % % % % % % % % % % % % % % %
%  		Load sample images        %
% % % % % % % % % % % % % % % % % %

data_dir = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/data/';

image_1 = imread(strcat(data_dir, 'scene_1_1.jpg'));
image_2 = imread(strcat(data_dir, 'scene_1_2.jpg'));

% % % % % % % % % % % % % % % % % %
%  		Camera Paramers           %
% % % % % % % % % % % % % % % % % %

K = [1043.57  0.0      0.0; 
     0.0      1043.57  0.0; 
     639.436  341.257  1.0]';

cameraParams = cameraParameters( 'IntrinsicMatrix', K');

% % % % % % % % % % % % % % % % % % % % %
%  	Recover [I|0] from identity image   %
% % % % % % % % % % % % % % % % % % % % %

% the question is how to test this ...
% problem: we don't have ground truth.
% we need to test what is the best
% kind of features to use to get the alignment
% if none available, we can start incorporating
% accelerameter data as anothony suggested
I1 = image_1;
I2 = image_2;


% determine correspondances between points
imagePoints1 = detectMinEigenFeatures(rgb2gray(I1), 'MinQuality', 0.1);
imagePoints2 = detectMinEigenFeatures(rgb2gray(I2), 'MinQuality', 0.1);

% % Visualize detected points
figure
imshow(I1, 'InitialMagnification', 50);
title('150 Strongest Corners from the First Image');
hold on
plot(selectStrongest(imagePoints1, 150));

% Create the point tracker
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

% Initialize the point tracker
imagePoints1 = imagePoints1.Location;
initialize(tracker, imagePoints1, I1);

% Track the points
[imagePoints2, validIdx] = step(tracker, I2);
matchedPoints1 = imagePoints1(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);

% Visualize correspondences
figure
showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);
title('Tracked Features');

% % % % % % % % % % % % % % % % % % % % % %
%    Determine [R|t] using image features
% % % % % % % % % % % % % % % % % % % % % %

% Estimate the fundamental matrix
[E, epipolarInliers] = estimateEssentialMatrix(...
    matchedPoints1, matchedPoints2, cameraParams, 'Confidence', 99.99);

% Find epipolar inliers
inlierPoints1 = matchedPoints1(epipolarInliers, :);
inlierPoints2 = matchedPoints2(epipolarInliers, :);

% Display inlier matches
figure
showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2);
title('Epipolar Inliers');

% note, rotation correct, translation is incorrect
[orient, loc] = relativeCameraPose(E, cameraParams, inlierPoints1, inlierPoints2);
	























