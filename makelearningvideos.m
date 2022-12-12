clear all
load /media/david/LaCieG/data-navioc/res_learnexp_acc2sum.mat
%load /media/david/LaCieG/data-navioc/res_learnexp_smabsacc.mat

addpath Visualization/
walls = vario('walls_diamor_1.mat', 'walls');
walls(5, 2) = 60;
walls(10, 2) = 60;
%% Set 1 (DIAMOR-train)

fprintf('\nSample    ')
for i = 1:length(samples_train)
	fprintf('\b\b\b')
	fprintf('%3i', i)
	mdp_data_arr_train{i}.sampling_time = 0.05;
	fig = playsample(samples_train{i}, mdp_data_arr_train{i}, false, ...
		[20, 60], [-2.5, 7.5], walls, ...
		sprintf('vid/diamor_train_samples_%02i.avi', i));
	if isgraphics(fig)
		close(fig);
	else
		break
	end
end
fprintf('\n')

fprintf('\nSample    ')
for i = 1:length(re_samples_train)
	fprintf('\b\b\b')
	fprintf('%3i', i)
	mdp_data_arr_train{i}.sampling_time = 0.05;
	fig = playsample(re_samples_train{i}, mdp_data_arr_train{i}, false, ...
		[20, 60], [-2.5, 7.5], walls, ...
		sprintf('vid/diamor_train_re_samples_acc2_%02i.avi', i));
	if isgraphics(fig)
		close(fig);
	else
		break
	end
end
fprintf('\n')

%% Set 2 (DIAMOR-test)
fprintf('\nSample    ')
for i = 1:length(samples_test)
	fprintf('\b\b\b')
	fprintf('%3i', i)
	mdp_data_arr_test{i}.sampling_time = 0.05;
	fig = playsample(samples_test{i}, mdp_data_arr_test{i}, false, ...
		[20, 60], [-2.5, 7.5], walls, ...
		sprintf('vid/diamor_test_samples_%02i.avi', i));
	if isgraphics(fig)
		close(fig);
	else
		break
	end
end
fprintf('\n')

fprintf('\nSample    ')
for i = 1:length(re_samples_test)
	fprintf('\b\b\b')
	fprintf('%3i', i)
	mdp_data_arr_test{i}.sampling_time = 0.05;
	fig = playsample(re_samples_test{i}, mdp_data_arr_test{i}, false, ...
		[20, 60], [-2.5, 7.5], walls, ...
		sprintf('vid/diamor_test_re_samples_acc2_%02i.avi', i));
	if isgraphics(fig)
		close(fig);
	else
		break
	end
end
fprintf('\n')

%% Set 3 (ETH)
fprintf('\nSample    ')
for i = 1:length(re_samples_eth)
	fprintf('\b\b\b')
	fprintf('%3i', i)
	mdp_data_arr_eth{i}.sampling_time = 0.4;
	fig = playsample(trajlets{i}, mdp_data_arr_eth{i}, false, ...
		[-5, 15], [-1, 11], [], ...
		sprintf('vid/eth_samples_%02i.avi', i));
	if isgraphics(fig)
		close(fig);
	else
		break
	end
end
fprintf('\n')

fprintf('\nSample    ')
for i = 1:length(re_samples_eth)
	fprintf('\b\b\b')
	fprintf('%3i', i)
	mdp_data_arr_eth{i}.sampling_time = 0.05;
	fig = playsample(re_samples_eth{i}, mdp_data_arr_eth{i}, false, ...
		[-5, 15], [-1, 11], [], ...
		sprintf('vid/eth_re_samples_acc2_%02i.avi', i));
	if isgraphics(fig)
		close(fig);
	else
		break
	end
end
fprintf('\n')

clear all
%load /media/david/LaCieG/data-navioc/res_learnexp_acc2sum.mat
load /media/david/LaCieG/data-navioc/res_learnexp_smabsacc.mat
addpath Visualization/
walls = vario('walls_diamor_1.mat', 'walls');
walls(5, 2) = 60;
walls(10, 2) = 60;
%% Set 1 (DIAMOR-train)
fprintf('\nSample    ')
for i = 1:length(re_samples_train)
	fprintf('\b\b\b')
	fprintf('%3i', i)
	mdp_data_arr_train{i}.sampling_time = 0.05;
	fig = playsample(re_samples_train{i}, mdp_data_arr_train{i}, false, ...
		[20, 60], [-2.5, 7.5], walls, ...
		sprintf('vid/diamor_train_re_samples_smabsacc_%02i.avi', i));
	if isgraphics(fig)
		close(fig);
	else
		break
	end
end
fprintf('\n')

%% Set 2 (DIAMOR-test)
fprintf('\nSample    ')
for i = 1:length(re_samples_test)
	fprintf('\b\b\b')
	fprintf('%3i', i)
	mdp_data_arr_test{i}.sampling_time = 0.05;
	fig = playsample(re_samples_test{i}, mdp_data_arr_test{i}, false, ...
		[20, 60], [-2.5, 7.5], walls, ...
		sprintf('vid/diamor_test_re_samples_smabsacc_%02i.avi', i));
	if isgraphics(fig)
		close(fig);
	else
		break
	end
end
fprintf('\n')

%% Set 3 (ETH)
fprintf('\nSample    ')
for i = 1:length(re_samples_eth)
	fprintf('\b\b\b')
	fprintf('%3i', i)
	mdp_data_arr_eth{i}.sampling_time = 0.05;
	fig = playsample(re_samples_eth{i}, mdp_data_arr_eth{i}, false, ...
		[-5, 15], [-1, 11], [], ...
		sprintf('vid/eth_re_samples_smabsacc_%02i.avi', i));
	if isgraphics(fig)
		close(fig);
	else
		break
	end
end
fprintf('\n')