function [sample, mdp_data, goals] = loadstates(fpath)

res = load(fpath);
X = res.X;

goals = res.goals;

n_agents = size(X, 2)/4;

sample = struct('s', X(1, :), 'states', X(2:end, :));

h = 0.05;

mdp_data = struct(...
	'time_step', h, ... [s]
	'n_ped', n_agents, ...
	'dims', 4*n_agents, ... positions, velocities
	'udims', 2*n_agents, ... accelerations
	'v_des', zeros(1, 2*n_agents), ...
	'wheelchair_companion_pair', logical(zeros(n_agents)), ...
	'wheelchair_pedestrian_pair', logical(zeros(n_agents)), ...
	'other_pair', logical(ones(n_agents)), ...
	'type', zeros(1, n_agents) ...
);