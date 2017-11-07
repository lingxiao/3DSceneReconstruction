
% @Use: convert video found at video_path and save at data_dir/video_name
%       if video already exist but overide, then reconvert video
function img_dir =  convert_video(data_dir, video_name, video_path, overide)

	img_dir    = strcat(data_dir, video_name, '/')

	if and(exist(img_dir) == 7, overide == 1)

		'path exists, skipping ....'

	else

		mkdir(img_dir)

		video = VideoReader(video_path);

		img_num = 1;

		while hasFrame(video)
			img = readFrame(video);
			filename = strcat(img_dir, 'frame-', num2str(img_num), '.jpg');
			imwrite(img, filename)
			img_num = img_num + 1;
		end	
	end

end
