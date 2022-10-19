function alltestswrite(stitch_only,create_sh,precompile,corn)

% Automatically generate all test scripts.
testwrite('locally_optimal_2D','objectworldtest(%s,%s,%s,0,''%s'')',4,5,8,stitch_only,create_sh,precompile,corn,0);
testwrite('globally_optimal_2D','objectworldtest(%s,%s,%s,1,''%s'')',4,5,8,stitch_only,create_sh,precompile,corn,0);
testwrite('linear_robot','robotarmtest(%s,%s,%s,0,''%s'')',4,5,8,stitch_only,create_sh,precompile,corn,0);
testwrite('nonlinear_robot','robotarmtest(%s,%s,%s,1,''%s'')',4,5,8,stitch_only,create_sh,precompile,corn,0);
testwrite('timing_robot','robotarmtiming(%s,%s,%s,''%s'')',4,7,8,stitch_only,create_sh,precompile,corn,0);
