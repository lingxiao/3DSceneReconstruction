
% @USE: given n x 3 matrix of point cloud, plot as vectors

function m = plot_cloud(cloud, title_xs)

	figure;
	rotate3d on;
	hold on;

	[m,n] = size(cloud);

	for k = 1:m
		v = cloud(k,:);
		quiver3(0,0,0,v(1), v(2), v(3));
	end

	title(title_xs)
	hold off;

end