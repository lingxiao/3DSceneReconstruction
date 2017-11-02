
% TODO: confirm the logic here with Stephen,
% it does not check out for some reason
function Rt = pick_sensible_Rt(point_1,point_2, intrinsicMatrix, R1_hat_1, R1_hat_2, t1_hat_1, t1_hat_2)

	xyz_1_1 = corresponding_point_to_3D(point_1, point_2, intrinsicMatrix, R1_hat_1, t1_hat_1');
	xyz_1_2 = corresponding_point_to_3D(point_1, point_2, intrinsicMatrix, R1_hat_1, t1_hat_2');
	xyz_2_1 = corresponding_point_to_3D(point_1, point_2, intrinsicMatrix, R1_hat_2, t1_hat_1');
	xyz_2_2 = corresponding_point_to_3D(point_1, point_2, intrinsicMatrix, R1_hat_2, t1_hat_2');

	% now pick the right [R|t]

	rotation_candidates = { {xyz_1_1, R1_hat_1, t1_hat_1}
	                      , {xyz_1_2, R1_hat_1, t1_hat_2}
	                      , {xyz_2_1, R1_hat_2, t1_hat_1}
	                      , {xyz_2_2, R1_hat_2, t1_hat_2}
	                      };
	rotations = {}

	% find the [R|t] candidates where y = R(x - t) is infront of camera
	for l = 1:length(rotation_candidates)

		tup = rotation_candidates{l};
		xyz = tup{1};
		R   = tup{2};
		t   = tup{3};

		if not(xyz < 0)
			rotations{length(rotations) + 1} = {xyz, R, t}
		end

	end	

	% find the [R|t] candidate where camera is facing forward after rotation
	z = [0 0 1];

	final = {};

	for l = 1:length(rotations)

		tup = rotations{l};
		xyz = tup{1};
		R   = tup{2}
		t   = tup{3}

		zR = z * R

		if zR(3) > 0
			final{length(final) + 1} = tup;
		end

	end

	% get the final rotation and translation
	R = final{1}{2};
	t = final{1}{3};

	Rt = [R t'];
