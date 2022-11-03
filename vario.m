function v = vario(mat_filepath_rel_to_root, var_name, variable)
filepath = strcat(getroot(), mat_filepath_rel_to_root);
if nargin == 3
	if iscell(variable)
		S = struct(var_name, {variable});
	else
		S = struct(var_name, variable);
	end
	save(filepath, '-struct', 'S');
else
	v = getfield(load(filepath), var_name);
end

function root = getroot()
root = '/media/gonond/LaCieG/data-navioc/';
if ~exist(root, 'dir')
	root = '/media/david/LaCieG/data-navioc/';
end