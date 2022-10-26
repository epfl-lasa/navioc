function fit_batch_vdes = addvdestobatch(i, j)

if nargin == 2
	s1 = sprintf('s_p00001/fit_batches/fit_batch_%i_%i.mat', i, j);
	s2 = sprintf('s_p00001/fit_batches_vdes/fit_batch_%i_%i.mat', i, j);
else
	s1 = sprintf('s_p00001/fit_batches/fit_batch_%i.mat', i);
	s2 = sprintf('s_p00001/fit_batches_vdes/fit_batch_%i.mat', i);
end

root = '/media/gonond/LaCieG/large-datasets-cri/';
fit_batch = getfield(load(strcat(root, s1)), 'fit_batch');

for i = 1:length(fit_batch.fits)
	if ~fit_batch.fits{i}.empty
		Vmag = sqrt(sum(fit_batch.fits{i}.fit.X_opt.^2, 2));
		v_mode = findmode(Vmag, [0.2, 2], 10);
		if isempty(v_mode)
			fit_batch.fits{i}.v_des = nan;
		else
			fit_batch.fits{i}.v_des = v_mode;
		end
	end
end

save(strcat(root, s2), 'fit_batch');