x_min = 0;
x_max = 60;
y_min = -10;
y_max = 10;

pos_cond = @(X) any(...
	X(:, 1:2:end) > x_min & ...
	X(:, 1:2:end) < x_max & ...
	X(:, 2:2:end) > y_min & ...
	X(:, 2:2:end) < y_max, 1);

exponent = -2;
samples = {};
for i = 1:6
	fit_batch = vario(sprintf('fit_batches_s1e%i/batch_%i.mat', exponent, i), 'fit_batch');
	samples = [samples, selectsamples(fit_batch, pos_cond, 97, -1, 7, 12)];
end

mdp_data_arr = cell(1, length(samples));
n_agents_arr = zeros(1, length(samples));

for i = 1:length(samples)

	n_agents = length(samples{i}.s)/4;
	n_agents_arr(i) = n_agents;

	delta_x = samples{i}.states(end, 1:2:(2*n_agents)) - ...
		samples{i}.states(1, 1:2:(2*n_agents));

	v_des_x = samples{i}.vxabs_des;
	v_des_x(delta_x < 0) = -v_des_x(delta_x < 0);

	%v_des = reshape([v_des_x; zeros(1, n_agents)], [1, 2*n_agents]);
	Vx = samples{i}.states(:, (1:2:(2*n_agents)) + 2*n_agents);
	Vy = samples{i}.states(:, (2:2:(2*n_agents)) + 2*n_agents);
	Vmag = Vx.^2 + Vy.^2;
	use_V_ref = mean(Vmag, 1) > 0.3;
	V_ref_tmp = reshape(mean(samples{i}.V_ref, 1), [2, n_agents]);
	V_ref_tmp(:, ~use_V_ref) = 0.0;
	V_ref = reshape(V_ref_tmp, [1, 2*n_agents]);

	wheelchair_companion_pair = logical(zeros(n_agents));
	wheelchair_pedestrian_pair = logical(zeros(n_agents));
	other_pair = logical(zeros(n_agents));
	for i_ = 1:n_agents
		for j_ = (i_ + 1):n_agents
			if ((samples{i}.type(i_) == 1 && samples{i}.type(j_) == 0) || ...
				(samples{i}.type(i_) == 0 && samples{i}.type(j_) == 1))
				wheelchair_pedestrian_pair(i_, j_) = true;
			elseif ((samples{i}.type(i_) == 1 && samples{i}.type(j_) == 3) || ...
					(samples{i}.type(i_) == 3 && samples{i}.type(j_) == 1))
				wheelchair_companion_pair(i_, j_) = true;
			else
				other_pair(i_, j_) = true;
			end
		end
	end
	idx_all = 1:n_agents;
	mdp_data_arr{i} = struct(...
		'time_step', 0.05, ... [s]
		'n_ped', n_agents, ...
		'dims', 4*n_agents, ... positions, velocities
		'udims', 2*n_agents, ... accelerations
		'v_des', V_ref, ...
		'vx_des', v_des_x, ...
		'vmag_des', samples{i}.vmag_des, ...
		'wheelchair_companion_pair', wheelchair_companion_pair, ...
		'wheelchair_pedestrian_pair', wheelchair_pedestrian_pair, ...
		'other_pair', other_pair, ...
		'idx_wheelchair', idx_all(samples{i}.type == 1), ...
		'idx_companion', idx_all(samples{i}.type == 3), ...
		'idx_other', idx_all(samples{i}.type == 0 | samples{i}.type == 2), ...
		'type',  samples{i}.type ...
	);
end

S = struct('samples', {samples}, 'mdp_data_arr', {mdp_data_arr});
%vario(sprintf('samples_data/diamor_1_s1e%i.mat', exponent), 'samples_data', S);

histogram(n_agents_arr);
title("Number of agents per sample")
pause(1)

addpath Visualization
walls = vario('walls_diamor_1.mat', 'walls');
fprintf('\nSample    ')
for i = 1:length(samples)
	fprintf('\b\b\b')
	fprintf('%3i', i)
	fig = playsample(samples{i}, mdp_data_arr{i}, false, ...
		[0, 60], [-10, 10], walls);
	if isgraphics(fig)
		close(fig);
	else
		break
	end
end
fprintf('\n')