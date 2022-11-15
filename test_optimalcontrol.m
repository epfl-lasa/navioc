addpath Features2
addpath Crowdworld2
addpath cioc/General
addpath cioc/Reward
addpath cioc/FastHess
addpath cioc/Utilities/minFunc
addpath cioc/Auglag
addpath cioc/Laplace

x = [-5, 0, 5, 0.1, 1.3, 0, -1.3, 0];
T = 96;
h = 0.05;
t = h*(1:T)';
u_1_init = [zeros(T, 1), -20*ones(T, 1)];
v_des = [1.3, 0, -1.3, 0];

tic
for i = 1:100
[Px, Py, Vx, Vy, Ax, Ay] = optimalcontrol(x, u_1_init(:, 1), u_1_init(:, 2), v_des, h);
end
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