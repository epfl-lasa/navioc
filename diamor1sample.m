x_min = 25;
x_max = 60;
y_min = 0;
y_max = 6;

pos_cond = @(X) any(...
	X(:, 1:2:end) > x_min & ...
	X(:, 1:2:end) < x_max & ...
	X(:, 2:2:end) > y_min & ...
	X(:, 2:2:end) < y_max, 1);

samples = {};
for i = 1:6
	samples = [samples, selectsamples(i, pos_cond, 97)];
end
%save('samples_diamor_1.mat', 'samples')

mdp_data_arr = cell(1, length(samples));
n_agents_arr = zeros(1, length(samples));

for i = 1:length(samples)

	n_agents = length(samples{i}.s)/6;
	n_agents_arr(i) = n_agents;

	delta_x = samples{i}.states(end, 1:2:(2*n_agents)) - ...
		samples{i}.states(1, 1:2:(2*n_agents));

	v_des_x = samples{i}.v_des;
	v_des_x(delta_x < 0) = -v_des_x(delta_x < 0);

	v_des = reshape([v_des_x; zeros(1, n_agents)], [1, 2*n_agents]);

	mdp_data_arr{i} = struct(...
		'time_step', 0.05, ... [s]
		'n_ped', n_agents, ...
		'dims', 6*n_agents, ... positions, velocities, accelerations
		'udims', 2*n_agents, ... jerks
		...'half_width', max(abs([x_min, x_max])), ... [m]
		...'half_height', max(abs([y_min, y_max])), ... [m]
		'x_des', {{v_des, zeros(size(v_des))}}, ...
		'v_des', v_des ...
	);
end

histogram(n_agents_arr);
title("Number of agents per sample")
pause(1)

addpath Visualization
walls = getfield(load('/media/gonond/LaCieG/large-datasets-cri/walls_diamor_1.mat'), 'walls');
fprintf('\nSample    ')
for i = 1:length(samples)
	fprintf('\b\b\b')
	fprintf('%3i', i)
	fig = playsample(samples{i}, mdp_data_arr{i}, false, ...
		[x_min, x_max], [y_min - 0.25, y_max + 0.25], walls);
	if isgraphics(fig)
		close(fig);
	else
		break
	end
end
fprintf('\n')