addpath Features2
addpath Crowdworld2
addpath cioc/General
addpath cioc/Reward
addpath cioc/FastHess
addpath cioc/Utilities/minFunc
addpath cioc/Auglag
addpath cioc/Laplace
addpath Visualization

T = 96;
h = 0.05;
t = h*(0:T)';

x = [-5, 0, 5, 0.1, 1.3, 0, -1.3, 0];
v_des = [1.3, 0, -1.3, 0];
u_prev = zeros(T, 4);

m = 400;
X = zeros(m, 8);
X(1, :) = x;
for i = 2:m
    [X(i, :), u_prev, mdp_data] = multi_agent_mpc(X(i-1, :), u_prev, v_des, h);
end

sample = struct('s', X(1, :), 'states', X(2:end, :));
playsample(sample, mdp_data)