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

% % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%  		TOY PROBLEM WITH MANY POINT AS POINT CLOUD      %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% % % % % % % % % % % % % % % % % %
% Point cloud and rotated version %
% % % % % % % % % % % % % % % % % %

% intrinsic matrix

intrinsicMatrix  = [ 1 0 8;
                     0 1 8;
                     0 0 1 ];

K = intrinsicMatrix;                     

radialDistortion = [0.0 0.0];
camParam = cameraParameters( 'IntrinsicMatrix', K', 'RadialDistortion', radialDistortion);

    
% 1. manually define the point cloud
point_cloud = [[0 0 1]
		       [1 2 3]
		       [1 3 1.5];
		       [1 0.5 1];
		       [0.5 8 1];
		       [0.2, 3, 2];
		       [3, 1, 3];
		       [4 5 1];
		       ];

[row,col] = size(point_cloud);

% this fails assert_E_corret_for_all_points_cloud
% when you do this points are project onto infinity
% rotate by 90 degrees around y is same as flipping 
% the camera down ???
R = rotz(pi/8) * rotx(pi/16);
t = [1 1 1]'  ; 

% % % % % % % % % % % % 
%  Project onto Image % 
% % % % % % % % % % % % 

im_1 = world_to_image(point_cloud, K, eye(3), zeros(3,1));
im_2 = world_to_image(point_cloud, K, R, t);

% % % % % % % % % % % % % % % % % % % % % % % % %
%  Construct Essential Matrix from Definition   %
% % % % % % % % % % % % % % % % % % % % % % % % %

E = skew(t) * R;

% decompose essential matrix into R_ and t_
[R_,t_] = relative_camera_pose(E, camParam, im_1, im_2);

% run a test
assert_R_correct = all(all(R - R_ < 1e-10));
assert_t_correct = assert_constant_multiple(t,t_');

assert_Rt_from_constructed_E_correct = all([assert_R_correct, assert_t_correct])

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%  Estimate Essential Matrix from 8 Point Correspondances   %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% we use the 8 point algorithm

% determine fundamental matrix
F_8point = det_F_normalized_8point( [im_1'; ones(1,row)], [im_2'; ones(1,row)] );
E_8point = K'* F_8point * K;

[R8, t8] = relative_camera_pose(E_8point, camParam, im_1, im_2);

assert_R8_correct = all(all(R - R8 < 1e-10));
assert_t8_correct = assert_constant_multiple(t, t8');
  
assert_Rt_from_8point_correct = all([ assert_R8_correct, assert_t8_correct ])

% % % % % % % % % % % % % % % % % % %
%    Point cloud feature matching   %
% % % % % % % % % % % % % % % % % % % 

% now it makes to get a real picture and get feature extactions
% so we need to test a variety of feature extractors

% we need to grab some image with ground truth [R|t]
% note the corresponding points may not be the same points
% due to noise, so we need RANSAC at some point 

% see proto_real_image.m



























