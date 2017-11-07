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
%  		Load sample video         %
% % % % % % % % % % % % % % % % % %

data_dir = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/data/videos/';

video_translation = strcat(data_dir, 'translate-1.mp4');
video_rotation    = strcat(data_dir, 'rotate-1.mp4');
video_rotation_90 = strcat(data_dir, 'rotate-90.mp4');

% convert video to images
translate_path    = convert_video(data_dir, 'translate-1', video_translation, 1);
rotation_path     = convert_video(data_dir, 'rotation-1' , video_rotation,    1);
rotation_path_90  = convert_video(data_dir, 'rotation-90', video_rotation_90, 1);

% % % % % % % % % % % % % % % % % % % % % % % %
%  		find correspondance between frames    %
% % % % % % % % % % % % % % % % % % % % % % % %

% cameraParams
K = [1043.57  0.0      0.0; 
     0.0      1043.57  0.0; 
     639.436  341.257  1.0]';

cameraParams = cameraParameters( 'IntrinsicMatrix', K');

% if it succeeds, then we know visual infomration alone is good enough to determine correspondances

% if it fails:
% 	1. correspondance bad/not enough -> incorporate acclerometer and point cloud
%   2. recovery is not working properly given correspondances -> use some other method for correspondance?

% let's look at: correspondances, [R|t]
% on rotation 90 degrees, video 
if false
	[Rotation rotations translations inliers] = exec_visual_odometry_on_video(rotation_path_90, cameraParams);
end	

% in conclusion, can't just use video, which is HD and arkit raw feed is worse. need to incorporate inertial data
% to make any progress.


% see if you can get real time skeleton tracking work for hackathon





















