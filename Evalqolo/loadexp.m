function res = loadexp(mat_filepath_rel_to_root)
rootdir = '/media/gonond/LaCieG/nov2022experiments/';
if ~exist(rootdir, 'dir')
	rootdir = '/media/david/LaCieG/nov2022experiments/';
end
res = load(strcat(rootdir, mat_filepath_rel_to_root));