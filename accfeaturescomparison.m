addpath Features2
addpath Crowdworld2
addpath cioc/General
addpath cioc/Reward
addpath cioc/FastHess
addpath cioc/Utilities/minFunc
addpath cioc/Auglag
addpath cioc/Laplace

x = [-5, 0, 5, 0.01, 1.3, 0, -1.3, 0];
T = 96;
h = 0.05;
t = h*(0:T)';
u_1_init = [zeros(T, 1), -10*ones(T, 1)];
v_des = [1.3, 0, -1.3, 0];

tic
[Px, Py, Vx, Vy, Ax, Ay] = optimalcontrol(x, u_1_init(:, 1), u_1_init(:, 2), v_des, h);
toc

subplot(2, 1, 1)
hold on
plot(Px, Py)
daspect([1,1,1])

subplot(2, 1, 2)
hold on
plot(t, Vx, 'b', 'DisplayName', 'Vx')
plot(t, Vy, 'g', 'DisplayName', 'Vy')
legend();

u = [Ax, Ay];
x = [Px(1), Py(1), Vx(1), Vy(1)];
states = [Px(2:end), Py(2:end), Vx(2:end), Vy(2:end)];

n_agents = 1;

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

r1 = acc2sumevalreward(struct('expec', 1, 'type_c', false, 'type_w', false, 'type_other', false), ...
    	mdp_data, x, u, states);

r2 = accsmabssumevalreward(struct('s', 10, 'expec', 1, 'type_c', false, 'type_w', false, 'type_other', false), ...
    	mdp_data, x, u, states);

%figure
%hold on
%plot(t(2:end), r1, 'k')
%plot(t(2:end), r2, 'r')