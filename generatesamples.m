n_agents = 7;

rng(4);

%v_des = 1.3*[1, 0, -1, 0, 0, 1, 0, -1]; % [vx1, vy1, ..., vx4, vy4]
v_des = reshape([1-2*(rand(1, n_agents) > 0.5); zeros(1, n_agents)], [1, 2*n_agents]);

mdp_data = struct(...
	'time_step', 0.05, ... [s]
	'n_ped', n_agents, ...
	'dims', 6*n_agents, ... positions, velocities, accelerations
	'udims', 2*n_agents, ... jerks
	'half_width', 8.0, ... [m]
	'half_height', 1.0, ... [m]
	'v_max', 1.7, ... [m/s]
	'accel_max', 5.0, ... [m/s^2]
	'u_sigma', 3.0, ... [m/s^3] standard deviation for sampling controls
	'state_sampling_method', 'corridor', ... or 'random'
...	'pos_sampling_min_dist', 0.4, ...
...	'R', 0.4, ...
	'v_des', v_des ...
);

mdp = 'crowdworld';

features_dyn = {... control-dependent features
	struct('type', 'jerk2sum', 'scaling', -1)
};

features_pt = {... state-dependent features
	struct('type', 'iesum'), ...
	struct('type', 'xerr2sum', 'order', 1, 'x_des', v_des), ...
    struct('type', 'xerr2sum', 'order', 2, 'x_des', zeros(size(v_des))), ...
    struct('type', 'aleft2sum'), ...
    struct('type', 'along2sum') ...
};

theta = [1, -0.5*ones(size(features_pt))];

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

[example_samples, test_samples] = sampleexamples(mdp_data, mdp, ...
	reward, test_params, verbose);

addpath cioc/Auglag
addpath cioc/Laplace

irl_result = amerun(struct(), mdp, mdp_data,...
    features_pt, features_dyn, example_samples, 4);

disp('Theta normalized')
disp(irl_result.reward.theta/max(abs(irl_result.reward.theta)))

plot_sample_ = @(sample) plot_sample(sample, mdp_data);

subplot(2, 1, 1)
plot_sample_(example_samples{1})
subplot(2, 1, 2)
plot_sample_(test_samples{1})

function plot_sample(sample, mdp_data)
    v_des = mdp_data.v_des;
    hold on
    for i = 1:mdp_data.n_ped
        X = sample.states(:, 2*i-1);
        Y = sample.states(:, 2*i);
        h = plot(X, Y, 'Marker', 'o');
        c = get(h, 'Color');
        quiver(X(end), Y(end), v_des(2*i-1), v_des(2*i), 'Color', c);
    end
    xlim(2*[-mdp_data.half_width, mdp_data.half_width])
    ylim(4*[-mdp_data.half_height, mdp_data.half_height])
    daspect([1,1,1])
end
