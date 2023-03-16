addpath Features2
addpath Crowdworld2
addpath cioc/General
addpath cioc/Reward
addpath cioc/FastHess
addpath cioc/Utilities/minFunc
addpath cioc/Auglag
addpath cioc/Laplace
addpath Visualization

[sample_vo, mdp_data_vo, goals_vo] = loadstates('states_vo.mat');
%[sample_ao, mdp_data_ao, goals_ao] = loadstates('states_ao_2.mat');

%playsample(sample_vo, mdp_data_vo)
%playsample(sample_ao, mdp_data_ao)

x = sample_vo.s;
n = length(x)/4;
goals = reshape(goals_vo', 1, 2*n);

% n = 6;
% phi = linspace(0, 2*pi*(n-1)/n, n);
% P = reshape([cos(phi); sin(phi)], 1, 2*n);
% V = -P*1.33;
% P = P*7.0;
% x = [P, V] + 0.1*rand(size([P, V]));
% 
% goals = -P;

T = 96;
h = 0.05;
u_prev = 0.1*rand(T, 2*n);

m = 400;
X = zeros(m, 4*n);
X(1, :) = x;
for i = 2:m
    diff = reshape(goals - X(i-1, 1:(2*n)), 2, n)';
    dist = sqrt(sum(diff.^2, 2));
    dist(dist < 0.5) = 0.5;
    v_des = reshape((diff./dist*1.33)', 1, 2*n);
    [X(i, :), u_prev, mdp_data] = multi_agent_mpc(X(i-1, :), u_prev, v_des, h);
end

sample = struct('s', X(1, :), 'states', X(2:end, :));
playsample(sample, mdp_data)