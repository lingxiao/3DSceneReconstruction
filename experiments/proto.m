%{
	eclect scripts to sanity check openCV implementation
%}


% we need to santity check the [R|t] output by openCV implementation 

% camera intrinsics
K = [[1043.57 0.0     0.0],
	 [0.0     1043.57 0.0],
	 [635.435 342.025 1.0]];

% load the images
data_dir = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/data';
im1_path = fullfile(data_dir, 'scene_2_1.jpg');
im2_path = fullfile(data_dir, 'scene_2_2.jpg');

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
I1 = undistortImage(readimage(images,1),cameraParams);
I2 = undistortImage(readimage(images,2),cameraParams);

I1gray = rgb2gray(I1);
I2gray = rgb2gray(I2);

% 
imPoints1 = detectSURFFeatures(I1gray);
imPoints2 = detectSURFFeatures(I2gray);





