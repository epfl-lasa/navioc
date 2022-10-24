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
	struct('type', 'iesum') ...
	struct('type', 'xerr2sum', 'order', 1) ...
    ...struct('type', 'xerr2sum', 'order', 2) ...
    struct('type', 'aleft2sum') ...
    struct('type', 'along2sum') ...
    struct('type', 'xerr4sum', 'order', 2) ...
    ...struct('type', 'aleft4sum') ...
    ...struct('type', 'along4sum') ...
};

sample = pairforged(1e-5, 0, 1.5);
addpath Visualization
figure
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


test_params = struct(...
	'training_sample_lengths', size(sample.u, 1), ... number of time steps in any training sample
	'test_samples', 0, ... number of test samples
	'training_samples', 1, ... number of training samples
	'test_restarts', 1, ... number of repetitions of local optimizations to generate a test sample
	'example_restarts', 1, ... number of repetitions of local optimizations to generate a training sample
	'test_optimal', false, ... are test samples globally optimal?
	'example_optimal', false, ... are training samples globally optimal?
	'example_recompute_optimal', false ... are re-optimized examples globally optimal?
);

[re_samples, ~] = resampleexamples(mdp_data_arr{1}, 'crowdworld', ...
	irl_result.reward, irl_result.reward, test_params, {sample}, {}, true);
figure
playsample(re_samples{1}, mdp_data_arr{1});

%mdp_data,mdp,reward,true_reward,test_params,old_examples,old_tests,verbose

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