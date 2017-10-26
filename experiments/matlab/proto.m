%{
	eclect scripts to sanity check openCV implementation
	/Applications/MATLAB_R2017b.app/bin/matlab -nodesktop -nosplash

%}

clc;
close all;

% % % % % % % % % % % % % % % % % % % % % % %
%  		ADD ROBOTICS PACKAGE PATH           %
% % % % % % % % % % % % % % % % % % % % % % %


kin_path1 = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/experiments/matlab/kinematics/kinematics/robotlinks';
kin_path2 = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/experiments/matlab/kinematics/kinematics/screws';
kin_path3 = '/Users/lingxiao/Documents/Spectrum/3DSceneReconstruction/experiments/matlab/kinematics/kinematics/utils';

addpath(kin_path1, kin_path2, kin_path3);


% % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%  		TOY PROBLEM WITH ONE POINT AS POINT CLOUD       %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% % % % % % % %
% Point Cloud %
% % % % % % % %

% point cloud of one point [x,y,z]
% note by convention z comes out of the camera plane, and plane is on (x,y)
% cloud = [1 0 0];  image point is undefined since point is on image plane
% cloud = [0 1 0];  image point is udnefeind since image is on image plane 
cloud = [0 0 1];    % image point is at [5,5] since the cloud is at the center


% % % % % % % % % % %
% Camera Intrinsics %
% % % % % % % % % % %

% run worldToImage
% 	- strategy: 
% 	1. get cameraParameter into matlab accepted format
%   2. feed into worldToImage
% Camera projects image onto 10x10 px image with focal length of 1
intrinsicMatrix  = [ 1 0 5;
                     0 1 5;
                     0 0 1 ]';

radialDistortion = [0.0 0.0];

camParam0 = cameraParameters( 'IntrinsicMatrix', intrinsicMatrix, 'RadialDistortion', radialDistortion, 'ImageSize', [10, 10]);

% % % % % % % % % 
% [R|t] = [I|0] % 
% % % % % % % % % 

% projection of cloud onto image at world coordinate
imagePoints = worldToImage(camParam0, eye(3,3), zeros(3,1), cloud);

% % % % % % % % 
% Translation % 
% % % % % % % % 

% translate camera by 1 in x direction
imagePoints_x_1 = worldToImage(camParam0, eye(3,3), [1 0 0], cloud);

% translate camera by 1 in y direction
imagePoints_y_1 = worldToImage(camParam0, eye(3,3), [0 1 0], cloud);

% translate camera by 1 in z direction <- should be the same
imagePoints_z_1 = worldToImage(camParam0, eye(3,3), [0 0 1], cloud);

% translate camera by 10 in x direction <- should no longer be in image
% note the image point is defined but cannot be displayed in the image
imagePoints_x_10 = worldToImage(camParam0, eye(3,3), [10 0 0], cloud);


% % % % %  % 
% Rotation % 
% % % % %  % 

% rotate by 90 degrees about x now the point cloud is behind the image
imagePoints_x_90 = worldToImage(camParam0, rotx(pi/2), zeros(3,1), cloud);

% rotate by 90 degrees about y, now the image cloud is behind image
imagePoints_y_90 = worldToImage(camParam0, roty(pi/2), zeros(3,1), cloud);

% rotate by 90 degrees about z, no change in image cloud
imagePoints_z_90 = worldToImage(camParam0, rotz(pi/2), zeros(3,1), cloud);

% rotate by 45 degrees about x axis, projection is shifted in y
imagePoints_x_45    = worldToImage(camParam0, rotx(pi/4) , zeros(3,1), cloud);
imagePoints_x_neg45 = worldToImage(camParam0, rotx(-pi/4), zeros(3,1), cloud);
imagePoints_y_45    = worldToImage(camParam0, roty(pi/4) , zeros(3,1), cloud);
imagePoints_y_neg45 = worldToImage(camParam0, roty(-pi/4), zeros(3,1), cloud);

% % % % % % % % % % % %  %
% Rotation & Translation %   
% % % % % % % % % % % %  %

imagePoints_Rt_1 = worldToImage(camParam0, rotx(pi/4), [1,0,0], cloud);
imagePoints_Rt_2 = worldToImage(camParam0, rotx(pi/4), [0,1,0], cloud);

% % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%  		TOY PROBLEM WITH MANY POINT AS POINT CLOUD      %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% % % % % % % % % % % % % % % % % %
% Point cloud and rotated version %
% % % % % % % % % % % % % % % % % %

% intrinsic matrix

intrinsicMatrix  = [ 1 0 8;
                     0 1 8;
                     0 0 1 ]';

radialDistortion = [0.0 0.0];

camParam = cameraParameters( 'IntrinsicMatrix', intrinsicMatrix, 'RadialDistortion', radialDistortion, 'ImageSize', [16, 16]);

% 1. define the point cloud

point_cloud = [[0 0 1];
		       [1 2 3]]
		       % [1 3 1.5];
		       % [1 0.5 1];
		       % [0.5 8 1];
		       % ]

% 2. project point cloud onto image plane

im_points_1 = worldToImage(camParam, eye(3,3), zeros(3,1), point_cloud)

% 3. rotate camera by pi/4 degrees, no translation and project onto image plane

R_true = rotx(pi/4)
t_true = [0.5, 0.5, 0.5]

im_points_2 = worldToImage(camParam, R_true, t_true, point_cloud)

% let's verify im_points_2 is correct by multiplying manually

im_ps_2 = intrinsicMatrix' * [1 2 3]'

% im_ps_2 = im_points_1 


% % % % % % % % % % % % % % % % % % % % % % 
%    Construct Essential Matrix manually  %
% % % % % % % % % % % % % % % % % % % % % %

% 4. construct essential matrix
E_true = cross(R_true, skew(t_true))

% 5. verify it satisifies epipolar constraints
%    pick two image points and verify yEy' = 0
y1 = [ im_points_1(1,:) 1 ]
y2 = [ im_points_2(1,:) 1 ]

asssert_zero = y1 * E_true * y2'




% % % % % % % % % % % % % % % % % % %
%    Point cloud feature matching   %
% % % % % % % % % % % % % % % % % % %

% 4. assume perfect matching between points and estimate essential matrix

% this is probably not passed in correctly
matched_points_1 = cornerPoints(image_points_1);
matched_points_2 = cornerPoints(image_points_2);



% so from E_true we should be able to get relativeCameraPose [R_true| lam * t_true]
% if we don't, then we either used the wrong matched_points_j and/or used the wrong function
% to find the relative camera pose

% one thing we can do is do this manually, and confirm we get the R_true back and lam * t_true

% % % % % % % % % % % % % % % % %  everything below is suspect % % % % % % % % % % % % % % % % % % % % % % 

% E does not match E_true
% note the way you passed matched_points do not make any sense
% [E, inliers, status] = estimateEssentialMatrix(matched_points_1, matched_points_1, camParam)

% 5. Recover [R|t] from essential matrix 
%  this is incorrect both with E_True and E
[R, t] = relativeCameraPose(E_true, camParam, matched_points_1, matched_points_2);

% 6. Confirm we have [R|t] = [R_true|0] 
% note right now it looks nothing like the original R_true
 

% 7. Run the process for multiple [R|t] hardcoded on hard coded pointcloud


% 8. Run the process for multiple [R|t] randomized on hard coded point cloud


% 9. Run the process for multiple [R|t] randomized on randomized point cloud














% RUN THROUGH THE PIPELINE BELOW WHERE POJRECTED POINTS ARE DEFINED TO BE MATCHED






























