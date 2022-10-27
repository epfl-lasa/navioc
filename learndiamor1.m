samples = getfield(load('samples_diamor_1.mat'), 'samples');

samples = samples(22:24);

mdp_data_arr = definemdpdata(samples);

[features_pt, features_dyn] = definefeatures();

addiocpaths();

irl_result = amerun(struct(), 'crowdworld', mdp_data_arr,...
    features_pt, features_dyn, samples, 4);

test_params = definetestparams();

re_samples = resampleall(samples, mdp_data_arr, irl_result.reward, test_params);

function re_samples = resampleall(samples, mdp_data_arr, reward, test_params)
	re_samples = cell(1, length(samples));
	for i = 1:length(samples)
		[re_samples(i), ~] = resampleexamples(mdp_data_arr{i}, 'crowdworld', ...
			reward, reward, test_params, {samples{i}}, {}, true);
	end
end

function test_params = definetestparams()
	test_params = struct(...
		'training_sample_lengths', 96, ...
		'test_samples', 0, ... 
		'training_samples', 1, ... 
		'test_restarts', 1, ... 
		'example_restarts', 1, ... 
		'test_optimal', false, ... 
		'example_optimal', false, ... 
		'example_recompute_optimal', false ... 
	);
end

function mdp_data_arr = definemdpdata(samples)
	mdp_data_arr = cell(1, length(samples));

	for i = 1:length(samples)
		n_agents = length(samples{i}.s)/6;

		delta_x = samples{i}.states(end, 1:2:(2*n_agents)) - ...
			samples{i}.states(1, 1:2:(2*n_agents));
		v_des_x = samples{i}.v_des;
		v_des_x(delta_x < 0) = -v_des_x(delta_x < 0);
		v_des = reshape([v_des_x; zeros(1, n_agents)], [1, 2*n_agents]);

		mdp_data_arr{i} = struct(...
			'time_step', 0.05, ... [s]
			'n_ped', n_agents, ...
			'dims', 6*n_agents, ... positions, velocities, accelerations
			'udims', 2*n_agents, ... jerks
			'x_des', {{v_des, zeros(size(v_des))}}, ...
			'v_des', v_des ...
		);
	end
end

function addiocpaths()
	addpath Features
	addpath Crowdworld
	addpath cioc/General
	addpath cioc/Reward
	addpath cioc/FastHess
	addpath cioc/Utilities/minFunc
	addpath cioc/Auglag
	addpath cioc/Laplace
end

function [features_pt, features_dyn] = definefeatures()
	features_dyn = {... control-dependent features
		struct('type', 'jerk2sum')
	};
	features_pt = {... state-dependent features
		...struct('type', 'iesum') ...
		...struct('type', 'dinv2sum') ...
		struct('type', 'dmrinv2sum') ...
		...struct('type', 'dgaussiansum', 'sigma', 0.5) ...
		...struct('type', 'dgaussiansum', 'sigma', 1.0) ...
		struct('type', 'xerr2sum', 'order', 1) ...
	};
end