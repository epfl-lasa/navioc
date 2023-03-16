function [x_new, u, mdp_data] = multi_agent_mpc(x, u_prev, v_des, h)

n_agents = length(x)/4;

T = size(u_prev, 1);
sample = struct('s', x, 'u', u_prev);

mdp_data = struct(...
	'time_step', h, ... [s]
	'n_ped', n_agents, ...
	'dims', 4*n_agents, ... positions, velocities
	'udims', 2*n_agents, ... accelerations
	'v_des', v_des, ...
	'wheelchair_companion_pair', logical(zeros(n_agents)), ...
	'wheelchair_pedestrian_pair', logical(zeros(n_agents)), ...
	'other_pair', logical(ones(n_agents)), ...
	'type', zeros(1, n_agents) ...
);

features = {... 
	...struct('expec', 1, 'type', 'acc2sum', 'type_c', false, 'type_w', false, 'type_other', false, 'w0', -1) ...
	struct('s', 10, 'expec', 1, 'type', 'accsmabssum', 'type_c', false, 'type_w', false, 'type_other', false, 'w0', -1) ...
	struct('expec', 1, 'type', 'verr2sum', 'type_c', false, 'type_w', false, 'type_other', false) ...
	struct('expec', 1, 'type', 'dgaussiansum', 'sigma', 0.5, 'skip_wp', false, 'skip_wc', false, 'skip_other', false) ...
	struct('expec', 1, 'type', 'iesum', 'eps2', 0.01, 'a', 25, 'R', 0.6, 'skip_wp', false, 'skip_wc', false, 'skip_other', false) ...
};
theta = [-1.0000   -7.5471   -0.0976   -0.6593];
% theta = [...
% 	-0.1 ...
% 	-1 ...
% 	-0.25 ...
% 	-1.0 ...
% ];
reward = struct('type', 'sum', 'theta', theta, 'features', {features});

test_params = struct(...
	...'fast_features', true, ...
	'training_sample_lengths', T, ...
	'test_samples', 0, ... 
	'training_samples', 1, ... 
	'test_restarts', 1, ... 
	'example_restarts', 1, ... 
	'test_optimal', false, ... 
	'example_optimal', false, ... 
	'example_recompute_optimal', false ... 
);

%tic
%[re_samples, ~] = resampleexamples(mdp_data, 'crowdworld', ...
%			reward, reward, test_params, {sample}, {}, true);
%toc

% Set up optimization options.
options = struct();
options.Method = 'lbfgs';
options.maxIter = 50;
options.MaxFunEvals = 1000;
options.display = 'on';
options.TolX = 1.0e-16;
options.TolFun = 1.0e-8;
options.Display = 'off';

for f = 1:length(reward.features)
	reward.features{f}.fast = true;
end

% Run minFunc.
tic
[u_, r] = minFunc(@(p)fastreward(p, x, mdp_data, 'crowdworld', ...
	reward), sample.u(:), options);
toc
u = reshape(u_, size(sample.u, 1), mdp_data.udims);
x_new = crowdworldcontrol(mdp_data, sample.s, u(1, :));



function [x_red, v_des_red] = reduce_crowd(x, v_des, n_ped_max, v_mag_min)

n_agents = length(x)/4;
n_ped = n_agents - 1;

if n_ped <= n_ped_max
	x_red = x;
	v_des_red = v_des;
else
	p_red = zeros(1, 2*(n_ped_max + 1));
	v_red = zeros(1, 2*(n_ped_max + 1));
	v_des_red = zeros(1, 2*(n_ped_max + 1));

	p_red(1:2) = x(1:2);
	v_red(1:2) = x(n_agents*2 + (1:2));
	v_des_red(1:2) = v_des(1:2);

	pos = reshape(x(3:(2*n_agents)), [2, n_ped]);
	relpos = pos - x(1:2)';
	v_rob = x(n_agents*2 + (1:2));
	v_mag_rob = sqrt(sum(v_rob.^2));
	if v_mag_rob > v_mag_min
		ex_rob = v_rob./v_mag_rob;
		ey_rob = [-ex_rob(2), ex_rob(1)];
		score = -abs(ey_rob*relpos);
		relpos_x = ex_rob*relpos;
		score(relpos_x > 0) = score(relpos_x > 0) - 0.25*relpos_x(:, relpos_x > 0);
		score(relpos_x<= 0) = score(relpos_x<= 0) + 3.0*relpos_x(:, relpos_x <= 0);	
	else
		score = -sum(relpos.^2);
	end
	[~, idx] = sort(score, 'descend');
	i_keep = idx(1:n_ped_max);
	jj = reshape([i_keep*2 - 1; i_keep*2], [1, 2*n_ped_max]);
	p_red(3:end) = x(2 + jj);
	v_red(3:end) = x(n_agents*2 + 2 + jj);	
	v_des_red(3:end) = v_des(2 + jj);

	x_red = [p_red, v_red];
end
