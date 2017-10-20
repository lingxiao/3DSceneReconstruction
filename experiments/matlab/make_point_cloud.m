% 
% @USE: given row, col and maximum depth, create syntetic point cloud of n points
% 
function cloud = make_point_cloud(row, col, depth, n)

	xs = rand_num(0,row  ,n);
	ys = rand_num(0,col  ,n);
	zs = rand_num(0,depth,n);

	cloud = [xs, ys, zs];

	% plot point cloud
	figure;
	rotate3d on;
	hold on;

	for k = 1:n
		v = cloud(k,:);
		quiver3(0,0,0,v(1), v(2), v(3));
	end

	title('synthetic point cloud')
	hold off;

return