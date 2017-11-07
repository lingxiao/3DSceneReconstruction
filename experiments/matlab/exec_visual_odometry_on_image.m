
% @Use: given cameraParameter, and two images, determine [R|t] and number of inlier points used
function [Rt numInliers] = exec_visual_odometry_on_image(cameraParams, I1, I2)

	% remove lense distortion
	I1 = undistortImage(I1, cameraParams);
	I2 = undistortImage(I2, cameraParams);

	% Detect feature points
	imagePoints1 = detectMinEigenFeatures(rgb2gray(I1), 'MinQuality', 0.1);

	% Create the point tracker
	tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

	% Initialize the point tracker
	imagePoints1 = imagePoints1.Location;
	initialize(tracker, imagePoints1, I1);

	% Track the points
	[imagePoints2, validIdx] = step(tracker, I2);
	matchedPoints1 = imagePoints1(validIdx, :);
	matchedPoints2 = imagePoints2(validIdx, :);


	% Estimate the fundamental matrix
	[E, epipolarInliers] = estimateEssentialMatrix(...
	    matchedPoints1, matchedPoints2, cameraParams, 'Confidence', 99.99);

	% Find epipolar inliers
	inlierPoints1  = matchedPoints1(epipolarInliers, :);
	inlierPoints2  = matchedPoints2(epipolarInliers, :);
	[numInliers d] = size(inlierPoints1);

	% my hacked version with 8-point algo from online
	if false
		F_8point      = det_F_normalized_8point( [inlierPoints1'; ones(1,numInliers)], [ inlierPoints2' ; ones(1,numInliers)] );
		K             = cameraParams.IntrinsicMatrix;
		E_8point      = K'* F_8point * K;
		[orient, loc] = relative_camera_pose(E_8point, cameraParams, inlierPoints1, inlierPoints2);
	else
		% matlab out of box function
		[orient, loc] = relativeCameraPose(E, cameraParams, inlierPoints1, inlierPoints2);
	end

	Rt = [orient loc'];

	% Display inlier matches
	% figure
	showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2);
	title('Epipolar Inliers');
