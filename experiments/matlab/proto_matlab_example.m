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

% % % % % % % % % % % % % % % % % % % % % % % % % %
%    Point cloud feature matching on real Images  %
% % % % % % % % % % % % % % % % % % % % % % % % % %

% Load precomputed camera parameters
load upToScaleReconstructionCameraParameters.mat
imageDir = fullfile(toolboxdir('vision'), 'visiondata','upToScaleReconstructionImages');


% load images
images = imageDatastore(imageDir);
I1 = readimage(images, 1);
I2 = readimage(images, 2);
figure
imshowpair(I1, I2, 'montage');
title('Original Images');

% remove lense distortion
I1 = undistortImage(I1, cameraParams);
I2 = undistortImage(I2, cameraParams);
figure
imshowpair(I1, I2, 'montage');
title('Undistorted Images');

% % % % % % % % % % % % % % % % % % % % % % % % % %
%    Find point correspondances between images    %
% % % % % % % % % % % % % % % % % % % % % % % % % %

% Detect feature points
imagePoints1 = detectMinEigenFeatures(rgb2gray(I1), 'MinQuality', 0.1);

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
%    Determine [R|t] using matlab method  %
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

[orient, loc] = relativeCameraPose(E, cameraParams, inlierPoints1, inlierPoints2);

% % % % % % % % % % % % % % % % % % % % %
%    Determine [R|t] using my method    %
% % % % % % % % % % % % % % % % % % % % %

[row,col] = size(inlierPoints1);

K = cameraParams.IntrinsicMatrix';

F_8point = det_F_normalized_8point( [inlierPoints1'; ones(1,row)], [ inlierPoints2' ; ones(1,row)] );

E_8point = K'* F_8point * K;

% conclusion: these two solutions matches the out of box solution reasonably well so it seems to work
% the difference might be due to the radial distortion
% note: using relativeCameraPose also works here
[R8, t8] = relative_camera_pose(E_8point, cameraParams, imagePoints1, imagePoints2);

% this breaks, even though we should be using inlier points, not image points,
% in the future just use relativeCameraPose
% [R8, t8] = relative_camera_pose(E_8point, cameraParams, inlierPoints1, inlierPoints2);

% imagePoints1, imagePoints2);

% % % % % % % % % % % % % % % % % % % % % % % % % %
%	Reconstruct 3-D locations of matching points  %
% % % % % % % % % % % % % % % % % % % % % % % % % %

% everything down here is for future use. 
% now that we have a 
if false

	% Detect dense feature points. Use an ROI to exclude points close to the
	% image edges.
	roi = [30, 30, size(I1, 2) - 30, size(I1, 1) - 30];
	imagePoints1 = detectMinEigenFeatures(rgb2gray(I1), 'ROI', roi, ...
	    'MinQuality', 0.001);

	% Create the point tracker
	tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

	% Initialize the point tracker
	imagePoints1 = imagePoints1.Location;
	initialize(tracker, imagePoints1, I1);

	% Track the points
	[imagePoints2, validIdx] = step(tracker, I2);
	matchedPoints1 = imagePoints1(validIdx, :);
	matchedPoints2 = imagePoints2(validIdx, :);

	% Compute the camera matrices for each position of the camera
	% The first camera is at the origin looking along the Z-axis. Thus, its
	% rotation matrix is identity, and its translation vector is 0.
	camMatrix1 = cameraMatrix(cameraParams, eye(3), [0 0 0]);

	% Compute extrinsics of the second camera
	[R, t] = cameraPoseToExtrinsics(orient, loc);
	camMatrix2 = cameraMatrix(cameraParams, R, t);

	% Compute the 3-D points
	% TODO MAYBE: BUG here that needs to be removed
	points3D = triangulate(matchedPoints1, matchedPoints2, camMatrix1, camMatrix2);

	% Get the color of each reconstructed point
	numPixels = size(I1, 1) * size(I1, 2);
	allColors = reshape(I1, [numPixels, 3]);
	colorIdx = sub2ind([size(I1, 1), size(I1, 2)], round(matchedPoints1(:,2)), ...
	    round(matchedPoints1(:, 1)));
	color = allColors(colorIdx, :);

	% Create the point cloud
	ptCloud = pointCloud(points3D, 'Color', color);

	% % % % % % % % % % % % %
	%	Plot 3D point cloud %
	% % % % % % % % % % % % %

	% Visualize the camera locations and orientations
	cameraSize = 0.3;
	figure
	plotCamera('Size', cameraSize, 'Color', 'r', 'Label', '1', 'Opacity', 0);
	hold on
	grid on
	plotCamera('Location', loc, 'Orientation', orient, 'Size', cameraSize, ...
	    'Color', 'b', 'Label', '2', 'Opacity', 0);

	% Visualize the point cloud
	pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
	    'MarkerSize', 45);

	% Rotate and zoom the plot
	camorbit(0, -30);
	camzoom(1.5);
			
	% Label the axes
	xlabel('x-axis');
	ylabel('y-axis');
	zlabel('z-axis')

	title('Up to Scale Reconstruction of the Scene');
end

