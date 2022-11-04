if ~exist('fits', 'var')
    tracks = importdata("eth", "obsmat_seq_eth.txt");

    %% compute non-causal fits
    fits = cell(1, length(tracks));
    t1 = inf;
    t2 = -inf;
    for j = 1:length(fits)
        t1 = min([t1, tracks{j}.t(1)]);
        t2 = max([t2, tracks{j}.t(end)]);
    end
    h = 0.05; % time step of each trajectory
    exponent = -2;
    s = 10.^exponent; % regularizer controlling smoothness
    N_max = 100; % maximum # of time steps per optimization
    dt_extra = 5.0; % look ahead to additional data when splitting trajectory optimization
    for j = 1:length(fits)
        fits{j} = trackfit(tracks{j}, t1, t2, h, s, N_max, dt_extra);
    end
end
%% estimate reference velocities
unique_goals = [
  -2.0000000e+01   5.8566027e+00
  -6.5902743e+00   6.5724367e-02
  -6.5553084e+00   1.1867515e+01
   1.5107171e+01   5.5659299e+00
];
C = ["k", "b", "g", "m"];
I_goal = zeros(length(fits), 1);
Speeds = zeros(length(fits), 1);
figure
hold on
for j = 1:length(fits)
	[Speeds(j), I_goal(j)] = estimspeedandgoal(fits{j}.fit, unique_goals, 0.3, 3.0, 16);
    %[fits{j}.Vref, fits{j}.i_max] = estimvref(fits{j}.fit, goals, 0.3, 3.0, 16);
    plot(fits{j}.fit.X_opt(:, 1), fits{j}.fit.X_opt(:, 2), C(I_goal(j)))
    plot(fits{j}.fit.X_opt(end, 1), fits{j}.fit.X_opt(end, 2), C(I_goal(j)), 'Marker', 'o')
end
daspect([1,1,1])

Goals = unique_goals(I_goal, :);

pt_count = 0;
t1 = inf;
t2 = -inf;
T = [];
Na = length(tracks);
for j = 1:Na
	t1 = min([t1, tracks{j}.t(1)]);
	t2 = max([t2, tracks{j}.t(end)]);
	pt_count = pt_count + length(tracks{j}.t);
	T = sort(unique([T; tracks{j}.t]));
end

Nt = length(T);
X = zeros(Nt, 4*Na);
Vref = zeros(Nt, 2*Na);
def = logical(zeros(Nt, Na));
%s_seq = cell(nt, 1);
for i = 1:Nt
	%pos = [];
	%vel = [];
	for j = 1:Na
		ind = tracks{j}.t == T(i);
		if any(ind)
			def(i, j) = true;
			jj_p = (2*j - 1):(2*j);
			jj_v = 2*Na + jj_p;
			X(i, jj_p) = [tracks{j}.x(ind), tracks{j}.y(ind)];
			X(i, jj_v) = [tracks{j}.vx(ind), tracks{j}.vy(ind)];
			e_goal = Goals(j, :) - X(i, jj_p);
			e_goal = e_goal./sqrt(sum(e_goal.^2, 2));
			Vref(i, jj_p) = e_goal*Speeds(j);
			%pos = [pos, tracks{j}.x(ind), tracks{j}.y(ind)];
			%vel = [vel, tracks{j}.vx(ind), tracks{j}.vy(ind)];
			pt_count = pt_count - 1;
		end
	end
	%s_seq{i} = [pos, vel];
end

fprintf('Remaining points count = %i (should be 0).\n', pt_count);

dt_pred = 4.8;
N_steps = 12;
dt = 0.4;
trajlets = {};
for i = 1:N_steps:(Nt - N_steps)
	%ii = (i + 1):(i + N_steps);
	if abs(T(i + N_steps) - T(i) - 4.8) < 1e-6
		def_sub = def(i:(i + N_steps), :);
		jj_ag = find(all(def_sub, 1));
		n_ag = length(jj_ag);
		if n_ag > 0
			jj_p = reshape([jj_ag*2 - 1; jj_ag*2], [1, 2*n_ag]);
			jj_v = 2*Na + jj_p;
			X_trajl = X(i:(i + N_steps), [jj_p, jj_v]);
			trajlets = [trajlets, struct(...
				's', X_trajl(1, :), ...
				'states', X_trajl(2:end, :), ...
				'v_des', Vref(i, jj_p) ...
			)];
		end
	end
end

AgentCounts = zeros(1, length(trajlets));
for i = 1:length(trajlets)
	AgentCounts(i) = length(trajlets{i}.s)/4;
end

figure
histogram(AgentCounts)
xlabel("Trajlet size")

save('eth_trajlets.mat', 'trajlets')


% %% compute causal state estimates by kalman filtering
% kf_estimates = cell(1, length(tracks));
% for j = 1:length(tracks)
% 	n_off = ceil((tracks{j}.t(1) - t1)/h);
% 	%T_est = tracks{j}.t;%
% 	T_est = (t1 + n_off*h):h:tracks{j}.t(end);
% 	Pos = [tracks{j}.x, tracks{j}.y];
% 	init_vel = [tracks{j}.vx(1), tracks{j}.vy(1)];
% 	X_est = kalmanfilter(Pos, tracks{j}.t, T_est, init_vel);
% 	kf_estimates{j} = struct('n_off', n_off, 'type', 'ped', ...
% 		'T_est', T_est, 'X_est', X_est);
% end
% 
% fit_batch = struct(...
% 	'window', [t1; t2], ...
% 	'source', 'eth', ...
% 	'fits', {fits}, ...
% 	'kf_estimates', {kf_estimates} ...
% );