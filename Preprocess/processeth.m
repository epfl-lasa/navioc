% tracks = importdata("eth", "obsmat_seq_eth.txt");
% 
% %% compute non-causal fits
% fits = cell(1, length(tracks));
% t1 = inf;
% t2 = -inf;
% for j = 1:length(fits)
% 	t1 = min([t1, tracks{j}.t(1)]);
% 	t2 = max([t2, tracks{j}.t(end)]);
% end
% h = 0.05; % time step of each trajectory
% exponent = -2;
% s = 10.^exponent; % regularizer controlling smoothness
% N_max = 100; % maximum # of time steps per optimization
% dt_extra = 5.0; % look ahead to additional data when splitting trajectory optimization
% for j = 1:length(fits)
% 	fits{j} = trackfit(tracks{j}, t1, t2, h, s, N_max, dt_extra);
% end

%% compute causal state estimates by kalman filtering
kf_estimates = cell(1, length(tracks));
for j = 1:length(tracks)
	n_off = ceil((tracks{j}.t(1) - t1)/h);
	%T_est = tracks{j}.t;%
	T_est = (t1 + n_off*h):h:tracks{j}.t(end);
	Pos = [tracks{j}.x, tracks{j}.y];
	init_vel = [tracks{j}.vx(1), tracks{j}.vy(1)];
	X_est = kalmanfilter(Pos, tracks{j}.t, T_est, init_vel);
	kf_estimates{j} = struct('n_off', n_off, 'type', 'ped', ...
		'T_est', T_est, 'X_est', X_est);
end

fit_batch = struct(...
	'window', [t1; t2], ...
	'source', 'eth', ...
	'fits', {fits}, ...
	'kf_estimates', {kf_estimates} ...
);