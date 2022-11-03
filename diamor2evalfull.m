d_max = 0.6;

res = {vario('res_ie.mat', 'res'), vario('res_no_ie.mat', 'res')};

%% resample for full diamor 2
samples_data = vario('samples_data/diamor_2_s1e-2.mat', 'samples_data');

setiocpaths();
test_params = definetestparams();

reward_ie = res{1}{1}.irl_result.reward;
reward_no_ie = res{2}{1}.irl_result.reward;

%re_samples_ie = resampleall(samples_data.samples, samples_data.mdp_data_arr, reward_ie, test_params);
%re_samples_no_ie = resampleall(samples_data.samples, samples_data.mdp_data_arr, reward_no_ie, test_params);
[D_ie_all, D_gt_ie_all] = evaldistrelpos(samples_data.samples, re_samples_ie);
[D_no_ie_all, D_gt_no_ie_all] = evaldistrelpos(samples_data.samples, re_samples_no_ie);

D_ie = D_ie_all(:, any(D_gt_ie_all < d_max, 1));
D_no_ie = D_no_ie_all(:, any(D_gt_no_ie_all < d_max, 1));

D = {D_ie, D_no_ie};

names = ["with $ \tilde E $ ", "without $ \tilde E $"];
C = ["k", "r"; "g", "c"];
h = 0.05;
t = (1:96)*h;
figure
hold on
for i = 1:2
	%D_train = [res{i}{1}.D_train, res{i}{2}.D_train];
	%D_test = [res{i}{1}.D_test, res{i}{2}.D_test];
	%plot_mean_std(t, D_train, C(i, 1), names(i)+"-train")
	plot_mean_std(t, D{i}, C(i, 1), names(i))
end
[D_cv_all, D_gt_cv_all] = evaldistrelposcv(samples_data.samples, h);
D_cv = D_cv_all(:, any(D_gt_cv_all < d_max, 1));
plot_mean_std(t, D_cv, 'm', "const-vel")
l = legend();
l.Interpreter = 'Latex';
xlabel('$ t $ [s]', 'Interpreter', 'Latex')
ylabel('FDE [m]', 'Interpreter', 'Latex')

function plot_mean_std(X, Y, c, s)
	Mu = mean(Y, 2);
	Sigma = std(Y, 0, 2);
	plot(X, Mu, c, 'DisplayName', s)
	plot(X, Mu + Sigma, c, 'HandleVisibility', 'off')
	plot(X, Mu - Sigma, c, 'HandleVisibility', 'off')
end

function re_samples = resampleall(samples, mdp_data_arr, reward, test_params)
	re_samples = cell(1, length(samples));
	for i = 1:length(samples)
		[re_samples(i), ~] = resampleexamples(mdp_data_arr{i}, 'crowdworld', ...
			reward, reward, test_params, {samples{i}}, {}, true);
	end
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