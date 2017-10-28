
% @Use: similar to matlab's worldToImage, projects pointcloud onto image
% @param
% 	point_cloud   : m x 3 point cloud
%   intrinscMatrix: [fx 0 ox 
% 				     0  fy oy
%                    0  0  1 ]
%   R             : 3x3 rotation matrix
%   t             : 3 x 1 translation vector
function Image = world_to_image(point_cloud, intrinsicMatrix, R, t)

	[num_points, col] = size(point_cloud);

	homo    = [point_cloud'; ones(1,num_points)];
	rotated = [R';t']' * homo;    

	% project onto image plane that is z = 1 
	rotated_homo = bsxfun(@rdivide, rotated, rotated(3,:));

	proj    = intrinsicMatrix * rotated;
	proj    = bsxfun(@rdivide, proj, proj(end,:));
	Image   = proj(1:end-1,:)';

end
