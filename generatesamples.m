rng(4);

n_agents_set = [7, 5];

mdp_data_arr = cell(1, 2);
example_samples = cell(1, 2);
test_samples = cell(1, 2);

for i = 1:2

	n_agents = n_agents_set(i);

	%v_des = 1.3*[1, 0, -1, 0, 0, 1, 0, -1]; % [vx1, vy1, ..., vx4, vy4]
	v_des = reshape([1-2*(rand(1, n_agents) > 0.5); zeros(1, n_agents)], [1, 2*n_agents]);

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
		'state_sampling_method', 'corridor', ... or 'random'
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
		struct('type', 'iesum'), ...
		struct('type', 'xerr2sum', 'order', 1), ...
	    struct('type', 'xerr2sum', 'order', 2), ...
	    struct('type', 'aleft2sum'), ...
	    struct('type', 'along2sum') ...
	};

	theta = [1, 0.5*ones(size(features_pt))];

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

	[example_samples_i, test_samples_i] = sampleexamples(mdp_data_i, mdp, ...
		reward, test_params, verbose);

	mdp_data_arr{i} = mdp_data_i;
	example_samples{i} = example_samples_i{1};
	test_samples{i} = test_samples_i{1};
end

example_samples_pert = cell(1, length(example_samples));
for i = 1:length(example_samples)
    example_samples_pert{i} = struct();
    example_samples_pert{i}.u = example_samples{i}.u + 0.05*randn(size(example_samples{i}.u));
    example_samples_pert{i}.s = example_samples{i}.s;
    example_samples_pert{i}.states = crowdworldcontrol(mdp_data_arr{i}, example_samples{i}.s, example_samples_pert{i}.u);
end

example_samples = example_samples_pert;

addpath cioc/Auglag
addpath cioc/Laplace

irl_result = amerun(struct(), mdp, mdp_data_arr,...
    features_pt, features_dyn, example_samples, 4);

disp('Theta normalized')
disp(irl_result.reward.theta/abs(irl_result.reward.theta(1)))

subplot(2, 2, 1)
plot_sample(example_samples{1}, mdp_data_arr{1})
subplot(2, 2, 2)
plot_sample(example_samples{2}, mdp_data_arr{2})
subplot(2, 2, 3)
plot_sample(test_samples{1}, mdp_data_arr{1})
subplot(2, 2, 4)
plot_sample(test_samples{2}, mdp_data_arr{2})

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
