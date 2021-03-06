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

radialDistortion = [0.0 0.0];
    
camParam = cameraParameters( 'IntrinsicMatrix', intrinsicMatrix', 'RadialDistortion', radialDistortion, 'ImageSize', [16, 16]);

% 1. define the point cloud
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

Ro = eye  (3,3);
to = zeros(3,1);

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% START HERE
% 
% roty seem to act up in a lot of tests for some reason ...
% strategy: make sure this works before packaging
% things up into a top level function
% it makes sense to think about what's going on geometrically
% and reason from there
% I think when you rotate around y, the point should
% be no longer visible, so some of this stuff
% might not make sense

% run through all the functions so far, clean them
% up and figure out why roty act so weird

R1 = roty(pi/2);
t1 = [1 1 1]';    % <- the essential matrix is changing wrt this

% START HERE
% 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

% 2. project point cloud onto image plane

im_points_1 = worldToImage(camParam, Ro', to, point_cloud);
im_points_2 = worldToImage(camParam, R1', t1, point_cloud);

% manually constructed values check out
if true
	im_2 = world_to_image(point_cloud, intrinsicMatrix, R1, t1);
	assert_proj_eq_im_points_2 = im_2 - im_points_2 < 1e-10;
end

% % % % % % % % % % % % % % % % % % % % % % % % %
%  Construct Essential Matrix from Definition   %
% % % % % % % % % % % % % % % % % % % % % % % % %

% 4. construct essential matrix
E = skew(t1) * R1;

% 5. assert it passes these tests
assert_zero_det = det(E) == 0;
assert_zero_tr  = sum(sum(2 * E * E' * E - trace(E * E') * E)) == 0;

% check svd of E
[U,S,V] = svd(E);

assert_UV_1 = det(U*V') - 1	 < 1e-10;

% we see it's a scalar multiple of [1 1 0]
lambda_S = diag(S);
assert_S_1_1_0 = lambda_S(1) - lambda_S(2) < 1e-10;

% verify it satisifies epipolar constraints
% pick two image points and verify yEy' = 0

assert_E_correct = [assert_zero_det assert_zero_det assert_UV_1 assert_S_1_1_0];

for k = 1:row
end

% so we know the Essential matrix we constructed must be true
assert_EssentialMat_correct_for_all_points_in_cloud = all(assert_E_correct)

% % % % % % % % % % % % % % % % % % %
%   Determine [R|t] from E          %
% % % % % % % % % % % % % % % % % % %

% we should be able to recover R and t from the points by hand
% strategy:
% 	1. do it by hand
%   2. assert matlab function behave as expected
%   3. if not, just use hand coded thing in your experiment

[R1_hat_1, R1_hat_2, t1_hat_1, t1_hat_2] = determine_Rt_from_E(E);

% check test -> so we have two rotation candidates, now we need
% a translation up to scale and pick out the right [R|t]                                                               
checkSol_1 = all(all(R1 - R1_hat_1 < 1e-10));
checkSol_2 = all(all(R1 - R1_hat_2 < 1e-10));

assert_rotation_sol_exists = any([checkSol_1, checkSol_2])

check_trans_1 = assert_constant_multiple(t1_hat_1, t1');
check_trans_2 = assert_constant_multiple(t1_hat_2, t1');

assert_trans_sol_exists = any([check_trans_1, check_trans_2])

% now we need to pick the sensible solution:
% we have to test location of a 3D point using 
% each of the four possible solutions

% 1. First run it on the actual [R|t] and see if you can
%    recover (x,y,z) to scale
k = 3;
point_1 = im_points_1(k,:);
point_2 = im_points_2(k,:);

% this is used for debugging only, take out later
xyz = corresponding_point_to_3D(point_1, point_2, intrinsicMatrix, R1, t1)

% conclusion: passed the test with true [R1|t1]
assert_constant_multiple_xyz = assert_constant_multiple(point_cloud(k,:), xyz')

% get the final rotation and translation
Rt = pick_sensible_Rt(point_1,point_2, intrinsicMatrix, R1_hat_1, R1_hat_2, t1_hat_1, t1_hat_2)

% these should be correct
rotateMat = Rt(:,1:3)
transVec  = Rt(:,4)

 




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%  Estimate Essential Matrix from Point Correspondances   %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% 8-point algorithm summary.
% determine the fundamental matrix from the constraint:
% 		u_1' F u_2 = 0                (1)
%  where u_1, u_2 are corresponding points in two images
%  rearrangin,g, we can rewrite (1) as:
% 		Af = 0                         (2)
%  where f is 9-vector containgin entries of matrix F, and A is the
%  set of contraints from (1). 
% A is often overspecified and rank deficient, we do least squares:
% find f so that
%  		argmin_{f} || Af|| 
% so that
% 		f'f = 1
% Since F is u' F u = 0, F is rank 2. so
% find the closed F' to F that is singular under frobenius norm.
% 
% right now: run the 8point algo package from web

% run 8 point algo
x1 = [im_points_1'; ones(1,row)];
x2 = [im_points_2'; ones(1,row)];

% so this one runs, now we need to make sure it outputs as intended
% ways to test it:
% 	1. run it on out of box data
%  	2. get essential matrix out of F and compare
%   3. 
F_algebraic = det_F_algebraic(x1,x1);
F_gold      = det_F_gold(x1,x2, 1, 1);
F_8point    = det_F_normalized_8point(x1,x2);

% we see this does not make any sense. 
K = intrinsicMatrix;

% this one does not work
E_algebraic = K'* F_algebraic *K;

% these two work up to the same scale when we normalize the F_gold
E_gold   = K'* F_gold *K;
E_8point = K'* F_8point *K;

% 8 point algorithm has the right signs wrt Rotation matrix
[R_gold_1, R_gold_2, t_gold_1, t_gold_2]         = determine_Rt_from_E(E_gold);
[R_8point_1, R_8point_2, t_8point_1, t_8point_2] = determine_Rt_from_E(E_8point);

% write some one off test to see that 8point is correct for all inputs
% what we see is that the soln is somewhere, but different depending
% on rot_ and degree of rotation

% NEED TODO: determine which [R|t] to pick from the decomposition
%            determine which one of gold or 8point to pick
assert_8point_rotation_1 = R1 ./ R_8point_1;
assert_8point_rotation_2 = R1 ./ R_8point_2;

% % % % % % % % % % % % % % % % % % %
%    Point cloud feature matching   %
% % % % % % % % % % % % % % % % % % % 

% After fixing the [R|t] ambiguity

% now run the pipeline with point correspondances on wild images

% % % % % % % % % % % % % % % % % % % % % % %  SUSPECT % % % % % % % % % % % % % % % % % % % % % % % % 

% matched_points_1 = cornerPoints(im_points_1);
% matched_points_2 = cornerPoints(im_points_2);

% This is way wrong <- need to figure out how to debug this .. either do 8 point thing manually 
% 	- using the wrong function
%   - the points are passed in incorrectly
%   - one solution: check using manual method and see if it's the same thing
% [E_hat, inliers, status] = estimateEssentialMatrix(matched_points_1, matched_points_1, camParam)


% now we have need to estimate the Essential matrix from corresponding points, 
% and run through the [R|t] decomposition that way


% % % % % % % % % % % % % % % % % % %
%    Point cloud feature matching   %
% % % % % % % % % % % % % % % % % % %

% 4. assume perfect matching between points and estimate essential matrix

% this is probably not passed in correctly
% matched_points_1 = cornerPoints(im_points_1);
% matched_points_2 = cornerPoints(im_points_2);

% E here should be reasonably close to E constructed by definition
% note the way you passed matched_points do not make any sense
% [E_hat, inliers, status] = estimateEssentialMatrix(matched_points_1, matched_points_1, camParam)

% 5. Recover [R|t] from essential matrix 
%  this is incorrect both with E_True and E
% [R, t] = relativeCameraPose(E_true, camParam, matched_points_1, matched_points_2);

% 6. Confirm we have [R|t] = [R_true|0] 
% note right now it looks nothing like the original R_true
 

% 7. Run the process for multiple [R|t] hardcoded on hard coded pointcloud


% 8. Run the process for multiple [R|t] randomized on hard coded point cloud


% 9. Run the process for multiple [R|t] randomized on randomized point cloud

% RUN THROUGH THE PIPELINE BELOW WHERE PROJECTED POINTS ARE DEFINED TO BE MATCHED
