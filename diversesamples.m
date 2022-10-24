n_agents = 2;
v_des = [1.3, 0, -1.3, 0];
mdp_data_arr = {struct(...
	'time_step', 0.05, ... [s]
	'n_ped', n_agents, ...
	'dims', 6*n_agents, ... positions, velocities, accelerations
	'udims', 2*n_agents, ... jerks
	'half_width', 8.0, ... [m]
	'half_height', 1.0, ... [m]
	'x_des', {{v_des, zeros(size(v_des))}}, ...
	'v_des', v_des ...
)};
features_dyn = {... control-dependent features
	struct('type', 'jerk2sum')
};
features_pt = {... state-dependent features
	struct('type', 'iesum'), ...
	struct('type', 'xerr2sum', 'order', 1), ...
    struct('type', 'xerr2sum', 'order', 2), ...
    struct('type', 'aleft2sum'), ...
    struct('type', 'along2sum') ...
};

sample = pairforged(1e-5, 0, 1);
addpath Visualization
playsample(sample, mdp_data_arr{1});

addpath Features
addpath Crowdworld
addpath cioc/General
addpath cioc/Reward
addpath cioc/FastHess
addpath cioc/Utilities/minFunc
addpath cioc/Auglag
addpath cioc/Laplace

irl_result = amerun(struct(), 'crowdworld', mdp_data_arr,...
    features_pt, features_dyn, {sample}, 4);

disp('Theta normalized')
disp(irl_result.reward.theta/abs(irl_result.reward.theta(1)))

function sample = pairforged(s, x_shift, y_scale)
	h = 0.05;
	T_opt = 0:h:10;

	T = [0, 2, 4, 6, 8, 10, 12]';
	Px = T*1.3;
	Py = [0, 0, 0.05, 0.25, 0.3, 0.275, 0.26]'*y_scale;

	res = intertrajectoryfit([Px, Py], T, T_opt, h, s, true);

	P = [res.X_opt, [15 + x_shift, -0.1] - res.X_opt];
	V = [res.V_opt, -res.V_opt];
	A = [res.A_opt, -res.A_opt];
	U = [res.U_opt, -res.U_opt];

	sample = struct(...
		's', [P(1, :), V(1, :), A(1, :)], ...
		'states', [P(2:end, :), V(2:end, :), A(2:end, :)], ...
		'u', U(2:end, :) ...
	);
end