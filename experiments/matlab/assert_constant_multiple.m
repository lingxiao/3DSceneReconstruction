function out = assert_constant_multiple(vector_3D, estimation_3D)

	constant_multiple_3D = vector_3D ./ estimation_3D;

	assert_constant_multiple_3D_1 = constant_multiple_3D(1) - constant_multiple_3D(2)  < 10e-10;
	assert_constant_multiple_3D_2 = constant_multiple_3D(1) - constant_multiple_3D(3)  < 10e-10;
	assert_constant_multiple_3D_3 = constant_multiple_3D(2) - constant_multiple_3D(3)  < 10e-10;
	assert_constant_multiple_3D = all([assert_constant_multiple_3D_1, assert_constant_multiple_3D_2, assert_constant_multiple_3D_3]);

	out = assert_constant_multiple_3D;

end	

