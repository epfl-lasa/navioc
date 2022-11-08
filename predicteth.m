trajlets = getfield(load('Preprocess/eth_trajlets.mat'), 'trajlets');

%trajlets = trajlets(1:2);

mdp_data_arr = cell(1, length(trajlets));
for i = 1:length(trajlets)
	n_agents = length(trajlets{i}.s)/4;
	mdp_data_arr{i} = struct(...
		'time_step', 0.05, ... [s]
		'sampling_time', 0.4, ... [s] 
		'n_ped', n_agents, ...
		'dims', 4*n_agents, ... positions, velocities
		'udims', 2*n_agents, ... accelerations
		'v_des', trajlets{i}.v_des, ...
		'wheelchair_companion_pair', logical(zeros(n_agents)), ...
		'wheelchair_pedestrian_pair', logical(zeros(n_agents)), ...
		'other_pair', logical(ones(n_agents)), ...
		'type', zeros(1, n_agents) ...
	);

	trajlets{i}.u = zeros(96, 2*n_agents);
end

%res_learn = vario('res_no_ie.mat', 'res');
res_learn = vario('res_sel_ie.mat', 'res');
reward = res_learn{2}.irl_result.reward;
for i = 1:length(reward.features)
    reward.features{i}.expec = 1;
end



setiocpaths();

re_samples = resampleall(trajlets, mdp_data_arr, reward, definetestparams());



addpath Visualization
fprintf('\nTrajlet    ')
for i = 1:length(trajlets)
	fprintf('\b\b\b')
	fprintf('%3i', i)
	fig = playsample(trajlets{i}, mdp_data_arr{i}, false, ...
		[-20, 15], [-0, 11], []);
	mdp_data_arr{i}.sampling_time = 0.05;
	fig2 = playsample(re_samples{i}, mdp_data_arr{i}, false, ...
		[-20, 15], [-0, 11], []);
	if ~(isgraphics(fig) || isgraphics(fig2))
		break
	end
	if isgraphics(fig)
		close(fig);
	end
	if isgraphics(fig2)
		close(fig2);
	end
end
fprintf('\n')



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