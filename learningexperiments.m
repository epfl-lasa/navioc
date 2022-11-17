R = 0.4;
features_dyn = {... 
	struct('s', 10, 'expec', 1, 'type', 'accsmabssum', 'type_c', false, 'type_w', false, 'type_other', false, 'w0', -1) ...
	...struct('expec', 1, 'type', 'acc2sum', 'type_c', false, 'type_w', false, 'type_other', false, 'w0', -1) ...
};
features_pt = {... 
	struct('expec', 1, 'type', 'verr2sum', 'type_c', false, 'type_w', false, 'type_other', false) ...
	struct('expec', 1, 'type', 'dgaussiansum', 'sigma', 0.5, 'skip_wp', false, 'skip_wc', false, 'skip_other', false) ...
	struct('expec', 1, 'type', 'iesum', 'eps2', 0.01, 'a', 25, 'R', R, 'skip_wp', false, 'skip_wc', false, 'skip_other', false) ...
};

setiocpaths();

samples_data = vario('samples_data/diamor_1_corr_s1e-2.mat', 'samples_data');
selection = vario('selection_diamor_1_corr_s1e-2.mat', 'selection');

selection_train = selection(1:2:end);
selection_test = selection(2:2:end);

samples_train = samples_data.samples(selection_train);
mdp_data_arr_train = samples_data.mdp_data_arr(selection_train);
samples_test = samples_data.samples(selection_test);
mdp_data_arr_test = samples_data.mdp_data_arr(selection_test);

speed_min = 0.3;
phi_ref_counts_train = phirefcount(mdp_data_arr_train, speed_min);
phi_ref_counts_test = phirefcount(mdp_data_arr_test, speed_min);

if ~exist('irl_result', 'var')
	irl_result = amerun(struct(), 'crowdworld', mdp_data_arr_train,...
	        features_pt, features_dyn, samples_train, 4);

	reward = irl_result.reward;

	re_samples_train = resampleall(samples_train, mdp_data_arr_train, reward);
	re_samples_test = resampleall(samples_test, mdp_data_arr_test, reward);
end

D_train = evaldist(samples_train, re_samples_train, 1, speed_min);
D_test = evaldist(samples_test, re_samples_test, 1, speed_min);

h = 0.05;
re_samples_train_cv = constvelresampleall(samples_train, h);
re_samples_test_cv = constvelresampleall(samples_test, h);
D_train_cv = evaldist(samples_train, re_samples_train_cv, 1, speed_min);
D_test_cv = evaldist(samples_test, re_samples_test_cv, 1, speed_min);

C_train_orig = evalcoll(samples_train, R);
C_test_orig = evalcoll(samples_test, R);
C_train = evalcoll(re_samples_train, R);
C_test = evalcoll(re_samples_test, R);
C_train_cv = evalcoll(re_samples_train_cv, R);
C_test_cv = evalcoll(re_samples_test_cv, R);

nc_train_orig = sum(C_train_orig);
nc_test_orig = sum(C_test_orig);
nc_train = sum(C_train);
nc_test = sum(C_test);
nc_train_cv = sum(C_train_cv);
nc_test_cv = sum(C_test_cv);

addpath Visualization
t = h*(1:96)';
figure
hold on
plotmeanquart(t, D_train, 'k', 'NavIRL - training set')
plotmeanquart(t, D_test, 'r', 'NavIRL - test set')
plotmeanquart(t, D_train_cv, [0.25, 0, 0.75], 'CV - training set')
plotmeanquart(t, D_test_cv, 'c', 'CV - test set')
legend()

trajlets = getfield(load('Preprocess/eth_trajlets.mat'), 'trajlets');

mdp_data_arr_eth = cell(1, length(trajlets));
h_eth = 0.4;
for i = 1:length(trajlets)
	n_agents = length(trajlets{i}.s)/4;
	mdp_data_arr_eth{i} = struct(...
		'time_step', h, ... [s]
		'sampling_time', h_eth, ... [s] 
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

if ~exist('re_samples_eth', 'var')
	re_samples_eth = resampleall(trajlets, mdp_data_arr_eth, reward);
end

D_eth = evaldist(trajlets, re_samples_eth, 8, speed_min);

re_samples_eth_cv = constvelresampleall(trajlets, h);
D_eth_cv = evaldist(trajlets, re_samples_eth_cv, 8, speed_min);

C_eth_orig = evalcoll(trajlets, R);
C_eth = evalcoll(re_samples_eth, R);
C_eth_cv = evalcoll(re_samples_eth_cv, R);

nc_eth_orig = sum(C_eth_orig);
nc_eth = sum(C_eth);
nc_eth_cv = sum(C_eth_cv);

t_eth = h*(8:8:96)';
figure
hold on
plotmeanquart(t_eth, D_eth, 'r', 'NavIRL')
plotmeanquart(t_eth, D_eth_cv, 'c', 'CV')
t_kretzschmar = [0, 1, 2, 3, 4]';
err_kretzschmar = [
	0
	1.6853	
	3.4051
	4.2995
	4.48
]/6.1833;
plot(t_kretzschmar, err_kretzschmar, 'ko-', 'DisplayName', 'Kretzschmar et al. (2016)')
xlabel('$\Delta t$ [s]', 'Interpreter', 'Latex')
ylabel('Error [m]', 'Interpreter', 'Latex')
legend()
title('ETH (test) set')

samples_data_train = struct(...
	'samples', {samples_train}, ...
	'mdp_data_arr', {mdp_data_arr_train});
samples_data_test = struct(...
	'samples', {samples_test}, ...
	'mdp_data_arr', {mdp_data_arr_test});
re_samples_data_train = struct(...
	'samples', {re_samples_train}, ...
	'mdp_data_arr', {mdp_data_arr_train});
re_samples_data_test = struct(...
	'samples', {re_samples_test}, ...
	'mdp_data_arr', {mdp_data_arr_test});
samples_data_eth = struct(...
	'samples', {trajlets}, ...
	'mdp_data_arr', {mdp_data_arr_eth});
re_samples_data_eth = struct(...
	'samples', {re_samples_eth}, ...
	'mdp_data_arr', {mdp_data_arr_eth});
features = [features_dyn, features_pt];
if ~exist('F_train', 'var')
	F_train_orig = fstats(features, samples_data_train);%, true, 'train orig -');
	F_test_orig = fstats(features, samples_data_test);%, true, 'test orig -');
	F_train = fstats(features, re_samples_data_train);%, true, 'train re -');
	F_test = fstats(features, re_samples_data_test);%, true, 'test re -');

	F_eth_orig = fstats(features, samples_data_eth);%, true, 'eth orig -');
	F_eth = fstats(features, re_samples_data_eth);%, true, 'eth re -');

	bounds = [0.07, 0.04, 0.3, 0.02];
	figure
	for j = 1:4
		subplot(4, 4, j)
		plotfstats(F_train_orig(:, j, :), strcat('Set 1-', features{j}.type), bounds(j), false)
	end
	for j = 1:4
		subplot(4, 4, j+4)
		plotfstats(F_train(:, j, :), strcat('Set 1 resampled-', features{j}.type), bounds(j), false)
	end
	for j = 1:4
		subplot(4, 4, j+8)
		plotfstats(F_test_orig(:, j, :), strcat('Set 2-', features{j}.type), bounds(j), false)
	end
	for j = 1:4
		subplot(4, 4, j+12)
		plotfstats(F_test(:, j, :), strcat('Set 2-resampled ', features{j}.type), bounds(j), false)
    end
    
% 	for j = 1:4
% 		subplot(6, 4, j+16)
% 		plotfstats(F_eth_orig(:, j, :), strcat('ETH-', features{j}.type), bounds(j), false)
% 	end
% 	for j = 1:4
% 		subplot(6, 4, j+20)
% 		plotfstats(F_eth(:, j, :), strcat('ETH resampled-', features{j}.type), bounds(j), false)
% 	end
% 	sigma_u = 0.5;
% 	rand_samples_data_train = resamplerand(samples_data_train, 10, sigma_u);
% 	rand_samples_data_test = resamplerand(samples_data_test, 10, sigma_u);
% 	F_train_rand = fstats(features, rand_samples_data_train, true, 'train rand -');
% 	F_test_rand = fstats(features, rand_samples_data_test, true, 'test rand -');
% 
% 	f_tilde_train_rand = mean13(F_train_rand, [1, 3]);
% 	f_tilde_test_rand = mean13(F_test_rand, [1, 3]);
% 	f_tilde_rand = mean13(cat(3, F_train_rand, F_test_rand), [1, 3]);
% 	
% 	f_tilde_train = mean13(F_train, [1, 3]);
% 	f_tilde_test = mean13(F_test, [1, 3]);
% 	f_tilde = mean13(cat(3, F_train, F_test), [1, 3]);
% 
% 	f_tilde_train_orig = mean13(F_train_orig, [1, 3]);
% 	f_tilde_test_orig = mean13(F_test_orig, [1, 3]);
% 	f_tilde_orig = mean13(cat(3, F_train_orig, F_test_orig), [1, 3]);

	f_80_orig = prctile(reshape(permute(cat(3, F_train_orig, F_test_orig), [2, 1, 3]), ...
		[length(features), 96*(length(selection_train) + length(selection_test))]), 80, 2);

	theta = reward.theta.*f_80_orig';

	% Nfeatures = features;
	% for i = 1:length(features)
	% 	Nfeatures{i}.expec = f_tilde_rand(i);
	% end
	% NF_train_orig = fstats(Nfeatures, samples_data_train, true, 'Norm. train orig -');
	% NF_test_orig = fstats(Nfeatures, samples_data_test, true, 'Norm. test orig -');
	% NF_train = fstats(Nfeatures, re_samples_data_train, true, 'Norm. train re -');
	% NF_test = fstats(Nfeatures, re_samples_data_test, true, 'Norm. test re -');
	% NF_train_rand = fstats(Nfeatures, rand_samples_data_train, true, 'Norm. train rand -');
	% NF_test_rand = fstats(Nfeatures, rand_samples_data_test, true, 'Norm. test rand -');
end

function re_samples = constvelresampleall(samples, h)
	re_samples = cell(1, length(samples));
	for i = 1:length(samples)
		[Nt, Nu] = size(samples{i}.u);
		P_0 = samples{i}.s(1:Nu);
		V_0 = samples{i}.s(Nu + (1:Nu));
		P_cv = P_0 + h*(1:Nt)'.*V_0;
		re_samples{i} = struct(...
			's', samples{i}.s, ...
			'states', [P_cv, repmat(V_0, [Nt, 1])], ...
			'u', zeros(Nt, Nu));
	end
end

function re_samples = resampleall(samples, mdp_data_arr, reward)
	test_params = definetestparams();
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

function counts = phirefcount(mdp_data_arr, speed_min)
	left = 0;
	up = 0;
	right = 0;
	down = 0;
	tol = 0.5*speed_min;
	for i = 1:length(mdp_data_arr)
		for j = 1:mdp_data_arr{i}.n_ped
			if mdp_data_arr{i}.v_des(j*2) > tol
				up = up + 1;
			elseif mdp_data_arr{i}.v_des(j*2) < -tol
				down = down + 1;
			elseif mdp_data_arr{i}.v_des(j*2 - 1) > tol
				right = right + 1;
			elseif mdp_data_arr{i}.v_des(j*2 - 1) < -tol
				left = left + 1;
			end
		end
	end
	counts = [left up right down];
end

function M = mean13(X, dummy)
	M = mean(mean(X, 1), 3);
end

function plotfstats(Fj, name, bound, legend_on)
    X = flat(Fj);
    Y = X(X < bound);
	hold on
	histogram(Y, 'HandleVisibility', 'off')
	xline(mean(X), "k", 'Mean')
	xline(median(X), "k--", 'Median')
	title(name)
	if legend_on
		legend()
	end
	if ~isinf(bound)
		xlim([0 bound])
	end
end


function xline(x, linespec, label)
    plot([x, x], getfield(gca(), 'YLim'), linespec, 'DisplayName', label)
end
function y = flat(X)
    y = reshape(X, [numel(X), 1]);
end