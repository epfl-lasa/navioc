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
u_1_init = zeros(T, 2);
v_des = [1.3, 0, -1.3, 0];

[Px, Py, Vx, Vy, Ax, Ay] = optimalcontrol(x, u_1_init(:, 1), u_1_init(:, 2), v_des);

plot(Px, Py)
daspect([1,1,1])