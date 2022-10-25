rng(4);

n_agents_set = [2];

mdp_data_arr = cell(1, 1);
example_samples = cell(1, 1);

for i = 1

	n_agents = n_agents_set(i);

	v_des = 1.3*[1, 0, -1, 0];

	mdp_data_i = struct(...
		'time_step', 0.05, ... [s]
		'n_ped', n_agents, ...
		'dims', 6*n_agents, ... positions, velocities, accelerations
		'udims', 2*n_agents, ... jerks
		'half_width', 8.0, ... [m]
		'half_height', 1.0, ... [m]
		'v_max', 1.7, ... [m/s]
		'accel_max', 5.0, ... [m/s^2]
		'u_sigma', 3.0, ... [m/s^3] standard deviation for sampling controls
		'state_sampling_method', 'pair', ... or 'random'
	...	'pos_sampling_min_dist', 0.4, ...
	...	'R', 0.4, ...
		'x_des', {{v_des, zeros(size(v_des))}}, ...
		'v_des', v_des ...
	);

	mdp = 'crowdworld';

	features_dyn = {... control-dependent features
		struct('type', 'jerk2sum')
	};

	features_pt = {... state-dependent features
        struct('type', 'iesum') ...
        struct('type', 'xerr2sum', 'order', 1) ...
        ...struct('type', 'activ', 'a', -5, 'c', 0, ...
        ...	'feature', struct('type', 'omega2sum')) ...
        struct('type', 'gaussian', 'mu', -0.1, 'sigma', 0.1, ...
        	'feature', struct('type', 'omega2sum')) ...
        struct('type', 'gaussian', 'mu', -0.3, 'sigma', 0.1, ...
        	'feature', struct('type', 'omega2sum')) ...
        ...struct('type', 'omega2sum')
        ...struct('type', 'aleft2sum') ...
        ...struct('type', 'along2sum') ...
        ...struct('type', 'xerr4sum', 'order', 2) ...
        ...struct('type', 'aleft4sum') ...
        ...struct('type', 'along4sum') ...
	};

	theta = [1, 1, 2, 1, -10];

	reward = struct(... linear combination (sum) of features, with weights theta
		'type', 'sum', ...
		'theta', theta, ...
		'features', {[features_dyn features_pt]} ...
	);

	test_params = struct(...
		'training_sample_lengths', 96, ... number of time steps in any training sample
		'test_samples', 0, ... number of test samples
		'training_samples', 1, ... number of training samples
		'test_restarts', 1, ... number of repetitions of local optimizations to generate a test sample
		'example_restarts', 3, ... number of repetitions of local optimizations to generate a training sample
		'test_optimal', false, ... are test samples globally optimal?
		'example_optimal', false ... are training samples globally optimal?
	);

	verbose = true;

	addpath Features
	addpath Crowdworld
	addpath cioc/General
	addpath cioc/Reward
	addpath cioc/FastHess
	addpath cioc/Utilities/minFunc

	[example_samples_i, ~] = sampleexamples(mdp_data_i, mdp, ...
		reward, test_params, verbose);

	mdp_data_arr{i} = mdp_data_i;
	example_samples{i} = example_samples_i{1};
end
addpath Visualization
figure
playsample(example_samples{1}, mdp_data_arr{1}, true);