fit_batch = vario('fit_batches_s1e-2/batch_1.mat', 'fit_batch');

hold on
for i = 1:length(fit_batch.fits)
	if ~fit_batch.fits{i}.empty
		plot(fit_batch.fits{i}.X(:, 1), fit_batch.fits{i}.X(:, 2), "k", 'LineWidth', 2)
    end
end
for i = 1:length(fit_batch.fits)
	if ~fit_batch.fits{i}.empty
		plot(fit_batch.fits{i}.fit.X_opt(:, 1), fit_batch.fits{i}.fit.X_opt(:, 2), "r", 'LineWidth', 1)
    end
end
daspect([1, 1, 1])