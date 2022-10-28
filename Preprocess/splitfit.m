function result = splitfit(X, T, T_opt, h, s, N_max, dt_extra)
n_segments = length(T_opt)/N_max;
if floor(n_segments) ~= n_segments
	n_segments = floor(n_segments) + 1;
end
if n_segments == 1
	result = intertrajectoryfit(X, T, T_opt, h, s);
	return
end
result = struct(...
	'X_est', zeros(size(X)), ...
	'T_opt', T_opt, ...
	'X_opt', zeros(length(T_opt), size(X, 2)), ...
	'V_opt', zeros(length(T_opt), size(X, 2)), ...
	'U_opt', zeros(length(T_opt), size(X, 2)));
for i = 1:n_segments
	if i == 1
		j_1 = 1;
		j_2 = N_max;
	elseif i == n_segments
		j_1 = (i - 1)*N_max; % overlap with last point
		j_2 = length(T_opt);
	else
		j_1 = (i - 1)*N_max; % overlap with last point
		j_2 = i*N_max;
	end
	T_opt_i = T_opt(j_1:j_2);
	k_1 = find(T < T_opt_i(1), 1, 'last');
	if isempty(k_1) || i == 1
		k_1 = 1;
	end
	k_2 = find(T > (T_opt_i(end) + dt_extra), 1);
	if isempty(k_2) || i == n_segments
		k_2 = length(T);
	end
	T_i = T(k_1:k_2);
	X_i = X(k_1:k_2, :);
	if i == 1
		res = intertrajectoryfit(X_i, T_i, T_opt_i, h, s);
	else
		res = intertrajectoryfit(X_i, T_i, T_opt_i, h, s, XV_opt_1);
	end
	XV_opt_1 = [res.X_opt(end, :), res.V_opt(end, :)]';
	% write partial result
	result.X_est(k_1:k_2, :) = res.X_est;
	result.X_opt(j_1:j_2, :) = res.X_opt;
	result.V_opt(j_1:j_2, :) = res.V_opt;
	result.U_opt(j_1:j_2, :) = res.U_opt;
end