function videobatch(i, j)
root = '/media/gonond/LaCieG/large-datasets-cri/';

if i <= 6
	walls = getfield(load(strcat(root, 'walls_diamor_1.mat')), 'walls');
else
	walls = [];
end

if nargin == 2
	s1 = sprintf('s_p00001/fit_batches/fit_batch_%i_%i.mat', i, j);	
else
	s1 = sprintf('s_p00001/fit_batches/fit_batch_%i.mat', i);
end
fit_batch = getfield(load(strcat(root, s1)), 'fit_batch');

s2 = sprintf('batches/batch_%i.mat', i);
batch = getfield(load(strcat(root, s2)), 'batch');

bins = bindata(batch.tracks, 20.0);
t1 = fit_batch.window(1) - bins.t_min;
t2 = fit_batch.window(2) - bins.t_min;

if i <= 11
	fig_limits = [0, 60, -20, 20];
else
	fig_limits = [-100, 100, -100, 100];
end

play(batch.tracks, fit_batch.fits, bins, fig_limits, t1, t2, walls)