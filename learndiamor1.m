samples_data = vario('samples_data_diamor_1.mat', 'samples_data');
samples = samples_data.samples;
mdp_data_arr = samples_data.mdp_data_arr;

[features_pt, features_dyn] = definefeatures();

setiocpaths();

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


function setiocpaths()
	orig_state = warning;
	warning('off','all');
	rmpath Features
	rmpath Crowdworld
	warning(orig_state);

	addpath Features2
	addpath Crowdworld2
	addpath cioc/General
	addpath cioc/Reward
	addpath cioc/FastHess
	addpath cioc/Utilities/minFunc
	addpath cioc/Auglag
	addpath cioc/Laplace
end

function [features_pt, features_dyn] = definefeatures()
	features_dyn = {... control-dependent features
		struct('type', 'acc2sum')
	};
	features_pt = {... state-dependent features
		struct('type', 'iesum', 'a', 25) ...
		...struct('type', 'dinv2sum') ...
		...struct('type', 'dmrinv2sum') ...
		...struct('type', 'dgaussiansum', 'sigma', 0.2) ...
		...struct('type', 'dgaussiansum', 'sigma', 0.45) ...
		struct('type', 'dgaussiansum', 'sigma', 0.5) ...
		struct('type', 'dgaussiansum', 'sigma', 1.5) ...
		...struct('type', 'dactivsum', 'a', 30, 'R', 0.3) ...
		struct('type', 'verr2sum') ...
	};
end