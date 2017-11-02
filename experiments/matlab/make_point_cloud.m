% 
% @USE: given row, col and maximum depth, create syntetic point cloud of n points
% 
function cloud = make_point_cloud(row, col, depth, n)

	xs = rand_num(0,row  ,n);
	ys = rand_num(0,col  ,n);

	zs = rand_num(0,depth,n);

	cloud = [xs, ys, zs];

return