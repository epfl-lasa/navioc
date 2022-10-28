function v = vario(mat_filepath_rel_to_root, var_name, variable)
filepath = strcat(getroot(), mat_filepath_rel_to_root);
if nargin == 3
	S = struct(var_name, variable);
	save(filepath, '-struct', 'S');
else
	v = getfield(load(filepath), var_name);
end

function root = getroot()
root = '/media/gonond/LaCieG/large-datasets-cri/';
if ~exist(root, 'dir')
	root = '/home/david/dataNavioc/';
end