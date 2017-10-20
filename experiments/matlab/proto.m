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


% DEFINE IMAGE SIZE AND MAXIMUM POINT DISTANCE FROM CAMERA
row   = 1000;
col   = 1000;
depth = 10  ;
num_points = 1;

% MAKE SYNTHETIC 3D POINT CLOUD

cloud = make_point_cloud(row,col,depth,num_points);

plot_cloud(cloud, 'original point cloud')

% ROTATE 3D POINT CLOUD BY [R|t]

% identity matrix
R1 = eye(3,3);

cloud_id = cloud * R1;

% rotate by 5 degrees around x axis
cloud_x_5 = cloud * rotx(90);
cloud_y_5 = cloud * roty(5);
cloud_z_5 = cloud * rotz(5);


plot_cloud(cloud_x_5, 'cloud rotated by 5 degrees about x')

% rotate by 10 degrees



% rotate by 20 degrees




% PROJECT 3D POINT CLOUD ONTO IMAGE PLANE


% RUN THROUGH THE PIPELINE BELOW WHERE POJRECTED POINTS ARE DEFINED TO BE MATCHED









return; 
% we need to santity check the [R|t] output by openCV implementation 

% camera intrinsics
K = [[1043.57 0.0     0.0],
	 [0.0     1043.57 0.0],
	 [635.435 342.025 1.0]];

% load the images
data_dir = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/data';
im1_path = fullfile(data_dir, 'scene_1_1.jpg');
im2_path = fullfile(data_dir, 'scene_1_2.jpg');

im1 = imread(im1_path);
im2 = imread(im2_path);

% SURF features points

% p1 = detectSURFFeatures(rgb2gray(im1));
% p2 = detectSURFFeatures(rgb2gray(im2));

% we need to determine the essential matrix from two RGB images 

% E = estimateEssentialMatrix(matchedPoints1, matchedPoints2, cameraParams)

% run existing example:

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

% feature extractor

% detect SURF features
% imPoints1 = detectSURFFeatures(I1gray);
% imPoints2 = detectSURFFeatures(I2gray);

imPoints1 = detectFASTFeatures(I1gray);
imPoints2 = detectFASTFeatures(I2gray);

% extract features
des1 = extractFeatures(I1gray, imPoints1, 'Upright', true);
des2 = extractFeatures(I2gray, imPoints2, 'Upright', true);

% match features
indexPairs = matchFeatures(des1, des2);

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

figure;
hold on;
rotate3d on;

u = [1 1 1]/norm([1 1 1 ])
v = u * relativeOrientation

quiver3(0,0,0,u(1),u(2),u(3));
quiver3(0,0,0,v(1),v(2),v(3));






















