
% 
% @Use: given essential matrix, cameraParameter object,
%       corresponding points from image 1 and 2, 
%       output estimated [R|t]
%       NOTE: this will not work if the rotation is more than pi/2 
%       about any axis
%       NOTE: this will not work if actual translation is negative 
function [R_ t_] = relative_camera_pose(E, camParam, im_1, im_2)

	% this gives the right R_, but the wrong t???
	[R_, t__] = relativeCameraPose(E, camParam, im_1, im_2);

	% now find the right t
	% bit of a hacky solution using my own thing
	[R1, R2, t1, t2] = determine_Rt_from_E(E);

	k = 2;
	P1 = im_1(k,:);
	P2 = im_2(k,:);

	M1 = [ R_ * [P1'; 1], [-P2'; 1], t1' ];
	[U1 S1 V1] = svd(M1);

	M2 = [ R_ * [P1'; 1], [-P2'; 1], t2' ];
	[U2 S2 V2] = svd(M2);

	if V1(3,3) > 0
		t_ = t1;
	else
		t_ = t2;
	end

end
