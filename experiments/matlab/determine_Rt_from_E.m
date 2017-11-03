% 
% @Use: Determine two rotation and translation from Essential Matrix
% 
function [R1, R2, t1, t2] =  determine_Rt_from_E(E)

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

	t1_hat_1 = unskew(S1);
	t1_hat_2 = unskew(S2);

	R1 = R1_hat_1;
	R2 = R1_hat_2;
	t1 = t1_hat_1;
	t2 = t1_hat_2;

end

