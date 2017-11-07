
% @Use: given cameraParameter, and path to images image_path, determine {[R|t]} and number of inlier points used for each pair of images
function [Rotation rotations translations inliers] = exec_visual_odometry_on_video(image_path, cameraParams)

	rotations    = {};
	translations = {};
	inliers      = {};

	Rotation = eye(3);

	paths = dir(image_path);
	[num_files m_] = size(paths);

	for k = 1:num_files

		path_1 = strcat(paths(k).folder, '/', 'frame-', num2str(k), '.jpg');
		path_2 = strcat(paths(k).folder, '/', 'frame-', num2str(k+1), '.jpg');
	
		if and(exist(path_2), and(contains(path_1, 'jpg'), contains(path_2, 'jpg')))
		
			I1 = imread(path_1);
			I2 = imread(path_2);

			[Rt numInliers] = exec_visual_odometry_on_image(cameraParams, I1, I2);

			R = Rt(:,1:3);
			t = Rt(: ,4 );

			rotations    {length(rotations)    + 1} = R;
			translations {length(translations) + 1} = t;
			inliers      {length(inliers) + 1}      = numInliers;

			Rotation = Rotation * R;

		end

		k = k + 1;

		% print current frame to console
		current_frame = k

	end

end



