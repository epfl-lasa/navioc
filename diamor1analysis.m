samples_data = vario(sprintf('samples_data/diamor_1_corr_s1e%i.mat', -2), 'samples_data');
%selection = vario('selection_diamor_1_corr_s1e-2.mat', 'selection');
features = {... 
	struct('type', 'acc2sum', 'type_c', false, 'type_w', false, 'type_other', false, 'w0', -1) ...
	struct('type', 'iesum', 'a', 25, 'R', 0.4, 'skip_wp', false, 'skip_wc', false, 'skip_other', false) ...
	struct('type', 'dgaussiansum', 'sigma', 0.5, 'skip_wp', false, 'skip_wc', false, 'skip_other', false) ...
	struct('type', 'dinv2sum', 'skip_wp', false, 'skip_wc', false, 'skip_other', false) ...
	struct('type', 'verr2sum', 'type_c', false, 'type_w', false, 'type_other', false) ...
};

disp('Evaluating features for actual samples')
F = fstats(features, samples_data);
disp('Computing random re-samples')
n_repeat = 10;
f_means_local = zeros(5, n_repeat);
f_means_median_local = zeros(5, n_repeat);
f_medians_local = zeros(5, n_repeat);
f_max_local = zeros(5, n_repeat);
f_means_max_local = zeros(5, n_repeat);
for i = 1:n_repeat
	rand_samples_data = resamplerand(samples_data, 10, 0.5);
	disp('Evaluating features for random re-samples')
	F_rand = fstats(features, rand_samples_data);
	for j = 1:5
		f_means_local(j, i) = mean(F_rand(:, j, :), 'all');
		f_means_median_local(j, i) = median(mean(F_rand(:, j, :), 1));
		f_medians_local(j, i) = median(F_rand(:, j, :), 'all');
		f_max_local(j, i) = max(F_rand(:, j, :), [], 'all');
		f_means_max_local(j, i) = max(mean(F_rand(:, j, :), 1));
	end
	clear F_rand
end
mean(f_means_local, 2)
std(f_means_local, 0, 2)