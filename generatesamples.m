n_agents = 4;

v_des = 1.3*[1, 0, -1, 0, 0, 1, 0, -1]; % [vx1, vy1, ..., vx4, vy4]

mdp_data = struct(...
	'time_step', 0.05, ... [s]
	'n_ped', n_agents, ...
	'dims', 6*n_agents, ... positions, velocities, accelerations
	'udims', 2*n_agents, ... jerks
	'half_width', 3.0, ... [m]
	'half_height', 3.0, ... [m]
	'v_max', 1.7, ... [m/s]
	'accel_max', 5.0, ... [m/s^2]
	'u_sigma', 3.0 ... [m/s^3] standard deviation for sampling controls
);

mdp = 'crowdworld';

features_dyn = {struct('type', 'jerk2sum')}; % control-dependent features

features_pt = {... state-dependent features
	struct('type', 'xerr2sum', 'order', 1, 'x_des', v_des) ...
};

theta = -[1, 1]; % penalize for |jerk|^2 and |v - v_des|^2

reward = struct(... linear combination (sum) of features, with weights theta
	'type', 'sum', ...
	'theta', theta, ...
	'features', {[features_dyn features_pt]} ...
);

test_params = struct(...
	'training_sample_lengths', 96, ... number of time steps in any training sample
	'test_samples', 1, ... number of test samples
	'training_samples', 1, ... number of training samples
	'test_restarts', 1, ... number of repetitions of local optimizations to generate a test sample
	'example_restarts', 1, ... number of repetitions of local optimizations to generate a training sample
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

[example_samples,test_samples] = sampleexamples(mdp_data, mdp, ...
	reward, test_params, verbose);

hold on
for i = 1:n_agents
    X = example_samples{1}.states(:, 2*i-1);
    Y = example_samples{1}.states(:, 2*i);
    h = plot(X, Y, 'Marker', 'o');
    c = get(h, 'Color');
    quiver(X(end), Y(end), v_des(2*i-1), v_des(2*i), 'Color', c);
end