
% USE: given row and col size of image, and 
%      vector of imagePoints in pixel location
%      output binary image where pixel at imagePoints
%      are painted one. 

function im = make_image(row,col,imagePoints)

	im = zeros(row,col);

	[num_points,xx] = size(imagePoints);

	for p = 1:num_points
		rc = imagePoints(p,:);
		r = round(rc(1));
		c = round(rc(2));
		if r <= row && c <= col && r > 0 && c > 0
			im(r,c) = 1;
		else
			r,c
	end

end

