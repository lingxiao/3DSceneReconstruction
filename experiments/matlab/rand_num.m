% 
% @Use: generate n random numbers within interval [min_r, max_r]
% 
function num = rand_num(min_r, max_r, n)

	num = (max_r - min_r) .* rand(n,1) + min_r;

end