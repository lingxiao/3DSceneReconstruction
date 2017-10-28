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

addpath(kin_path1, kin_path2, kin_path3);

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

R1 = rotx(pi/4);
t1 = [1 1 1]';    % <- the essential matrix is changing wrt this

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

assert_E_correct = [];

for k = 1:row
	y1 = [ im_points_1(k,:) 1 ] * inv(intrinsicMatrix');
	y2 = [ im_points_2(k,:) 1 ] * inv(intrinsicMatrix');

	% this is only zero when rotation = Identity
	% this is failing right now for all other cases
	assert_epipole_constraint = y2 * E * y1' < 1e-10;

	% right now all tests pass so our Essential matrix is correct
	assert_E_correct(end+1) = all([assert_zero_det assert_zero_det assert_UV_1 assert_S_1_1_0 assert_epipole_constraint]);
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

[U S V] = svd(E);

W = [ [0 -1 0];
	  [1  0 0];
	  [0  0 1]];

Z = [[0  1 0];
     [-1 0 0];
     [0  0 0]
    ];


% find two possible rotation matrices 
R1_hat_1 = U * W' * V';
R1_hat_2 = U * W * V';

% find possible S so that E = SR
S1 = (-1*U)*Z*U';
S2 = U*Z*U';

% now we can get translation up to scale
t1_hat_1 = unskew(S1);
t1_hat_2 = unskew(S2);


% check test -> so we have two rotation candidates, now we need
% a translation up to scale and pick out the right [R|t]                                                               
checkSol_1 = all(all(R1 - R1_hat_1 < 1e-10));
checkSol_2 = all(all(R1 - R1_hat_2 < 1e-10));

assert_rotation_sol_exists = any([checkSol_1, checkSol_2])

% now we have four possible [R|t], we need to get the right pair

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%  Estimate Essential Matrix from Point Correspondances   %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

matched_points_1 = cornerPoints(im_points_1);
matched_points_2 = cornerPoints(im_points_2);

% This is way wrong <- need to figure out how to debug this .. either do 8 point thing manually 
% 	- using the wrong function
%   - the points are passed in incorrectly
%   - one solution: check using manual method and see if it's the same thing
[E_hat, inliers, status] = estimateEssentialMatrix(matched_points_1, matched_points_1, camParam)

% try the solution: implement 8 point algorithm manually





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






























