[features_pt, features_dyn] = definefeaturesNOTnormalized();
setiocpaths();
test_params = definetestparams();

exponent = -2;
samples_data = vario(sprintf('samples_data/diamor_1_corr_s1e%i.mat', exponent), 'samples_data');
%selection = [23, 24]; %
selection = vario('selection_diamor_1_corr_s1e-2.mat', 'selection');
% selection = [];
% for i = 1:length(samples_data.samples)
%     if ~any(samples_data.mdp_data_arr{i}.type == 3)
%         selection = [selection, i];
%     end
% end
%selection = 1:length(samples_data.samples); %[6:10, 13:15, 20, 22:29];
%selection = [1:5, 11:12, 16:19, 21, 30:34];
res = {struct(), struct()};
for i = 2
    selection_test = selection(i:2:end);
    selection_train = selection((3-i):2:end);
    res{i}.samples_train = samples_data.samples(selection_train);
    res{i}.mdp_data_arr_train = samples_data.mdp_data_arr(selection_train);
    res{i}.samples_test = samples_data.samples(selection_test);
    res{i}.mdp_data_arr_test = samples_data.mdp_data_arr(selection_test);

    res{i}.irl_result = amerun(struct(), 'crowdworld', res{i}.mdp_data_arr_train,...
        features_pt, features_dyn, res{i}.samples_train, 4);

    res{i}.re_samples_train = resampleall(res{i}.samples_train, res{i}.mdp_data_arr_train, res{i}.irl_result.reward, test_params);
    res{i}.re_samples_test = resampleall(res{i}.samples_test, res{i}.mdp_data_arr_test, res{i}.irl_result.reward, test_params);

    res{i}.D_train = evaldist(res{i}.samples_train, res{i}.re_samples_train);
    res{i}.D_test = evaldist(res{i}.samples_test, res{i}.re_samples_test);
end

save /media/gonond/LaCieG/data-navioc/res_sel_ie.mat

% hold on
% plot((1:96)*0.05, mean(D_test, 2), 'r')
% plot((1:96)*0.05, mean(D_train, 2), 'k')
% 
% reward_0_ie = irl_result.reward;
% reward_0_ie.theta(2) = 0;
% re_samples_train_0_ie = resampleall(samples_train, mdp_data_arr_train, reward_0_ie, test_params);
% re_samples_test_0_ie = resampleall(samples_test, mdp_data_arr_test, reward_0_ie, test_params);
% 
% D_train_0_ie = evaldist(samples_train, re_samples_train_0_ie);
% D_test_0_ie = evaldist(samples_test, re_samples_test_0_ie);

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


function [features_pt, features_dyn] = definefeaturesNOTnormalized()

	features_dyn = {... 
		struct('expec', 1, 'type', 'acc2sum', 'type_c', false, 'type_w', false, 'type_other', false, 'w0', -1) ...
	};
	features_pt = {... 
		struct('expec', 1, 'type', 'iesum', 'eps2', 0.01, 'a', 25, 'R', 0.4, 'skip_wp', false, 'skip_wc', false, 'skip_other', false) ...
		struct('expec', 1, 'type', 'dgaussiansum', 'sigma', 0.5, 'skip_wp', false, 'skip_wc', false, 'skip_other', false) ...
		...struct('expec', 1, 'type', 'dgaussiansum', 'sigma', 1.5, 'skip_wp', false, 'skip_wc', false, 'skip_other', false) ...
		...struct('expec', 1, 'type', 'dvgaussiansum', 'sigma_p', 0.5, 'sigma_v', 0.5, 'skip_wp', false, 'skip_wc', false, 'skip_other', false) ...
		...struct('expec', 1, 'type', 'dinv2sum', 'skip_wp', false, 'skip_wc', false, 'skip_other', false) ...
		struct('expec', 1, 'type', 'verr2sum', 'type_c', false, 'type_w', false, 'type_other', false) ...
	};
end