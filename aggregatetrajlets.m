function agg = aggregatetrajlets(group, T)
n_traj = length(group.trajlets);
n_frag = length(group.fragments);
n = n_traj + n_frag;
N = 2*n;
agg = struct(...
	'X', zeros(T, N), ...
	'V', zeros(T, N), ...
	'A', zeros(T, N), ...
	'U', zeros(T, N), ...
	'def', logical([ones(T, n_traj), zeros(T, n_frag)]), ...
	'type', zeros(1, n), ...
	'v_des', zeros(1, n), ...
	'n', n, ...
	'n_traj', n_traj, ...
	'n_frag', n_frag);
for i = 1:n_traj
	jj = (2*i - 1):(2*i);
	agg.X(:, jj) = group.trajlets{i}.X;
	agg.V(:, jj) = group.trajlets{i}.V;
	agg.A(:, jj) = group.trajlets{i}.A;
	agg.U(:, jj) = group.trajlets{i}.U;
	if (strcmp(group.trajlets{i}.type, "powered_wheelchair") || ...
		strcmp(group.trajlets{i}.type, "manual_wheelchair"))
		agg.type(i) = 1;
	elseif strcmp(group.trajlets{i}.type, "stroller")
		agg.type(i) = 2;
	end
	agg.v_des(i) = group.trajlets{i}.v_des;
end
for i = 1:n_frag
	j = n_traj + i;
	jj = n_traj*2 + ((2*i - 1):(2*i));
	kk = group.fragments{i}.k_off + (1:size(group.fragments{i}.X, 1));
	agg.def(kk, j) = true;
	agg.X(kk, jj) = group.fragments{i}.X;
	agg.V(kk, jj) = group.fragments{i}.V;
	agg.A(kk, jj) = group.fragments{i}.A;
	agg.U(kk, jj) = group.fragments{i}.U;
	if (strcmp(group.fragments{i}.type, "powered_wheelchair") || ...
		strcmp(group.fragments{i}.type, "manual_wheelchair"))
		agg.type(j) = 1;
	elseif strcmp(group.fragments{i}.type, "stroller")
		agg.type(j) = 2;
	end
	agg.v_des(j) = group.fragments{i}.v_des;
end