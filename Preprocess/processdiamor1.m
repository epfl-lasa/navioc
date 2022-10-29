addpath ..
%% cut original data around wheelchair users' time windows
if ~exist('batches', 'var')
	try
		batches = vario('batches.mat', 'batches'); % load batches
	catch ME
		fprintf(2, ME.message);
		batches = cut_DIAMOR_ATC(30.0);
		vario('batches.mat', 'batches', batches); % save batches
	end
end
try
	vario('batches/batch_1.mat', 'batch');
catch ME
	for i = 1:length(batches) % also save each batch separately
	    vario(sprintf('batches/batch_%i.mat', i), 'batch', batches{i});
	end
end
%% fit trajectories to original data
n_b = 6; % diamor 1 %:11 diamor 1+2
fit_batches = cell(1, n_b);
for i = 1:n_b
	figure(i)
	hold on
	t1 = batches{i}.window(1);
	t2 = batches{i}.window(2);
	fits = cell(1, length(batches{i}.tracks));
	for j = 1:length(batches{i}.tracks)
		h = 0.05; % time step of each trajectory
		exponent = -2;
		s = 10.^exponent; % regularizer controlling smoothness
		N_max = 100; % maximum # of time steps per optimization
		dt_extra = 5.0; % look ahead to additional data when splitting trajectory optimization
		fits{j} = trackfit(batches{i}.tracks{j}, t1, t2, h, s, N_max, dt_extra);
		if ~fits{j}.empty
			idxA = batches{i}.tracks{j}.t >= fits{j}.fit.T_opt(1);
			if any(idxA)
				idxA = [idxA(2:end); true];
			end
			idxB = batches{i}.tracks{j}.t <= fits{j}.fit.T_opt(end);
			if any(idxB)
				idxB = [true; idxB(1:(end-1))];
			end
			idx = idxA & idxB;
			plot(batches{i}.tracks{j}.x(idx), batches{i}.tracks{j}.y(idx), "k--", 'LineWidth', 2)
			plot(fits{j}.fit.X_opt(:, 1), fits{j}.fit.X_opt(:, 2), "r")
		end
	end
	daspect([1, 1, 1])
	fit_batches{i} = struct(...
		'window', batches{i}.window, ...
		'source', batches{i}.source, ...
		'fits', {fits} ...
	);
	vario(sprintf('fit_batches_s1e%i/batch_%i.mat', exponent, i), 'fit_batch', fit_batches{i});
end