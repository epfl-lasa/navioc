fit_batch = vario('fit_batches_s1e-2/batch_2.mat', 'fit_batch');

hold on
for i = 1:5:length(fit_batch.fits)
	if ~fit_batch.fits{i}.empty
		plot(fit_batch.fits{i}.X(:, 1), fit_batch.fits{i}.X(:, 2), "k")
		plot(fit_batch.fits{i}.fit.X_opt(:, 1), fit_batch.fits{i}.fit.X_opt(:, 2), "r")
	end
end
daspect([1, 1, 1])