n = 10;
phi = linspace(0, 2*pi*(n-1)/n, n);
P = reshape([cos(phi); sin(phi)], 1, 2*n);
V = -P*1.33;
P = P*8.0;
x = [P, V] + 0.1*rand(size([P, V]));

goals = -P;

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