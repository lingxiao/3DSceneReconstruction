% 
% @Use: given:
%       - point_1 in image 1 and point_2 in image 2, 
%         in pixel coordinates
%       - intrinscMatrix of form:
% 			  fx 0  ox 
%             0  fy oy
%             0  0   1 
%     	- rotation martrix R    
%       - translation vector t
%           
%           
%        output 3D point in world coordinate
% 
% 
function xyz = corresponding_point_to_3D(point_1, point_2, intrinsicMatrix, R, t)

	x_1 = point_1(1);
	y_1 = point_1(2);

	x_2 = point_2(1);
	y_2 = point_2(2);

	r1 = R(1,:);
	r3 = R(3,:);

	top = (r1 - x_2 * r3) * t;
	bot = (r1 - x_2 * r3) * ([ point_1 1]');

	z_1 = top/bot;

	xyz = inv(intrinsicMatrix) *  (z_1 * [ point_1 1 ])';

	% there's a problem here in that three out of two solutions are giving solutions where z is positive
	% the four solutions should corresond, R1_2_hat should be the right solution
	% how do you get camera vector out of this ???

	% for now we normalize the z, so that translation is in effect
	% implictly normalized
	% TODO: tomorrow ask Stephen how to pick out the right xyz
	% in principle, especially the rotation part of it
	% xyz = xyz / xyz(3);

