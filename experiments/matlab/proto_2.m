
%{
	eclect scripts to sanity check openCV implementation
	/Applications/MATLAB_R2017b.app/bin/matlab -nodesktop -nosplash

%}


clc;
close all;

kin_path1 = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/experiments/matlab/kinematics/kinematics/robotlinks'
kin_path2 = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/experiments/matlab/kinematics/kinematics/screws'
kin_path3 = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/experiments/matlab/kinematics/kinematics/utils'

addpath(kin_path1, kin_path2, kin_path3)


% PROJECT 3D POINT CLOUD ONTO IMAGE PLANE
% 
% K = [ fx 0  ox 
%       0  fy oy 
%       0  0  1 ]
% 
% K = [[1043.57 0.0     0.0],
	 % [0.0     1043.57 0.0
	 % [635.435 342.025 1.0]]';


% RUN THROUGH THE PIPELINE BELOW WHERE POJRECTED POINTS ARE DEFINED TO BE MATCHED


% load the images
data_dir = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/data';
im1_path = fullfile(data_dir, 'scene_3_1.jpg');
im2_path = fullfile(data_dir, 'scene_3_2.jpg');

im1 = imread(im1_path);
im2 = imread(im2_path);

% SURF features points

% p1 = detectSURFFeatures(rgb2gray(im1));
% p2 = detectSURFFeatures(rgb2gray(im2));

% we need to determine the essential matrix from two RGB images 

% E = estimateEssentialMatrix(matchedPoints1, matchedPoints2, cameraParams)

% run existing example

load upToScaleReconstructionCameraParameters.mat

imageDir = fullfile(toolboxdir('vision'),'visiondata', 'upToScaleReconstructionImages');
images   = imageDatastore(imageDir);

% todo: see what happens to [R|t] when we don't run undistorImages
% I1 = undistortImage(readimage(images,1),cameraParams);
% I2 = undistortImage(readimage(images,2),cameraParams);

% I1 = readimage(images,1);
% I2 = readimage(images,2);

I1 = im1;
I2 = im2;

I1gray = rgb2gray(I1);
I2gray = rgb2gray(I2);

I2 = I1;

% feature extractor

% detect SURF features
% imPoints1 = detectSURFFeatures(I1gray);
% imPoints2 = detectSURFFeatures(I2gray);

% detect features and get pixel location
imPoints1 = detectFASTFeatures(I1gray);
imPoints2 = detectFASTFeatures(I2gray);

% extract features at pixel locations
des1 = extractFeatures(I1gray, imPoints1, 'Upright', true);
des2 = extractFeatures(I2gray, imPoints2, 'Upright', true);

% match descriptors from the two points
indexPairs = matchFeatures(des1, des2);

% pick out the pixel location of matched points
matchedP1 = imPoints1(indexPairs(:,1));
matchedP2 = imPoints2(indexPairs(:,2));

figure; 
showMatchedFeatures(I1, I2, matchedP1, matchedP2);
title('putative points')

% now estimate the esssential matrix
[E, inliers] = estimateEssentialMatrix(matchedP1, matchedP2, cameraParams);

inlierPoints1 = matchedP1 (inliers);
inlierPoints2 = matchedP2 (inliers);

figure
showMatchedFeatures(I1,I2,inlierPoints1,inlierPoints2);
title('Inlier Matches')

% compute relativeOrientation
[relativeOrientation,relativeLocation] = relativeCameraPose(E,cameraParams,inlierPoints1,inlierPoints2)

% for visualization let's plot a vector (1,1,1) in 3D
% note the outcome looks reasonable, even though we're using the 
% wrong cameraParams!!

