%% initializing 
clc; clear; close all;
addpath ./Features2;
addpath ./Crowdworld2;
addpath ./Utilities;

%% loading data
load('./Data/diamor_1_s1e-2.mat');
load('./Data/walls_diamor_1.mat');
samples = samples_data.samples;
mdp_data_arr = samples_data.mdp_data_arr;
walls_num = size(walls, 1);
x1 = walls(:, 1);
x2 = walls(:, 2);
y1 = walls(:, 3);
y2 = walls(:, 4);

%% bulding obstacles
Nobs = 9;
obstacles = cell(1, Nobs);
% defining points associated with each obstacle
obstacles{1} = struct('points',...
                      [x1(1), x2(1), x2(17);...
                       y1(1), y2(1), y2(17)]);
obstacles{2} = struct('points',...
                      [x1(2), x2(2), x2(18), x2(19);...
                       y1(2), y2(2), y2(18), y2(19)]);
obstacles{3} = struct('points',...
                      [x1(3), x2(3), x2(20), x2(21);...
                       y1(3), y2(3), y2(20), y2(21)]);
obstacles{4} = struct('points',...
                      [x1(4), x2(4), x2(22), x2(23);...
                       y1(4), y2(4), y2(22), y2(23)]);
obstacles{5} = struct('points',...
                      [x1(5), x2(5), x2(24);...
                       y1(5), y2(5), y2(24)]);
obstacles{6} = struct('points',...
                      [x1(6), x1(7), x2(7), x2(11);...
                       y1(6), y1(7), y2(7), y2(11)]);
obstacles{7} = struct('points',...
                      [x1(8), x2(8), x2(12), x2(13);...
                       y1(8), y2(8), y2(12), y2(13)]);
obstacles{8} = struct('points',...
                      [x1(9), x2(9), x2(14), x2(15);...
                       y1(9), y2(9), y2(14), y2(15)]);
obstacles{9} = struct('points',...
                      [x1(10), x2(10), x2(16);...
                       y1(10), y2(10), y2(16)]);
% sorting the points in ccw
for j = 1:Nobs
    % with sort_coords
    obstacles{j}.points = sort_coords(obstacles{j}.points, true);
    if size(obstacles{j}.points, 2) == 2
        obstacles{j}.num = size(obstacles{j}.points, 2)-1; % num is the total number of edges
    elseif size(obstacles{j}.points, 2) > 2
        obstacles{j}.num = size(obstacles{j}.points, 2); % num is the total number of edges
    end
    % with cnvhull
    % k = convhull(obstacles{j}.points(1, :), obstacles{j}.points(2, :));
    % obstacles{j}.points = obstacles{j}.points(:, k);
    % obstacles{j}.num = size(obstacles{j}.points, 2)-1;  % num is the total number of edges
end
% calculating the normals 
for j = 1:Nobs
    obstacles{j}.normals = zeros(2, obstacles{j}.num);
    % with sort_coords
    temp = obstacles{j}.points;
    temp = [temp, temp(:, 1)];
    for k = 2:obstacles{j}.num+1
        direction = temp(:, k) - temp(:, k-1);
        obstacles{j}.normals(:, k-1) = [0, 1; -1, 0] * direction; 
    end
    % with cnvhull
    % for k = 2:obstacles{j}.num+1
    %     direction = obstacles{j}.points(:, k) - obstacles{j}.points(:, k-1);
    %     obstacles{j}.normals(:, k-1) = [0, 1; -1, 0] * direction;
    % end
    obstacles{j}.normals = normc(obstacles{j}.normals);
end
% calculating the offsets
for j = 1:Nobs
    obstacles{j}.offsets = zeros(1, obstacles{j}.num);
    for k = 1:obstacles{j}.num
        obstacles{j}.offsets(k) = -1 * obstacles{j}.normals(:, k)' *...
                                  obstacles{j}.points(:, k);
    end
end
% adding obstacles to mdp_data_arr
for i = 1:size(mdp_data_arr, 2)
    mdp_data_arr{i}.obstacles = obstacles;
end

%% plotting the obstacles
gcf = figure;
hold on;
grid on;
box on;
axis equal;
xlim([-5, 60]);
ylim([-10, 15]);
for j = 1:Nobs
    % with sort_coords
    temp = obstacles{j}.points;
    temp = [temp, temp(:, 1)];
    plot(temp(1, :), temp(2, :), 'k-');
    for k = 1:obstacles{j}.num
        midpoint = (temp(:, k)+temp(:, k+1)) / 2; 
        normal = obstacles{j}.normals(:, k);
        quiver(midpoint(1), midpoint(2), normal(1), normal(2), 0,...
               'linewidth', 1, 'color', 'r');
    end
    % with cnvhull
    % plot(obstacles{j}.points(1, :), obstacles{j}.points(2, :), 'k-');
    % for k = 1:obstacles{j}.num
    %     midpoint = (obstacles{j}.points(:, k)+obstacles{j}.points(:, k+1)) / 2; 
    %     normal = obstacles{j}.normals(:, k);
    %     quiver(midpoint(1), midpoint(2), normal(1), normal(2), 0,...
    %            'linewidth', 1, 'color', 'r');
    % end
end
set(gca, 'TickLabelInterpreter', 'latex');
% print(gcf, './Figures/map.pdf', '-dpdf', '-r0', '-fillpage');

%% verifying the feature
% clc;
s = 10;
eps_ = 1e-10;
reward = struct('expec', 1, 's', s, 'eps', eps_, 'fast', 1,...
                'type', 'obstaclessum', 'type_c', false, 'type_w', false,...
                'type_other', false);
i = 1;  % choosing the desired cell from samples and mdp_data_arr
x = samples{i}.s;
u = samples{i}.u;
mdp_data = mdp_data_arr{i};
dx = 1e-5;  
[states, A, B] = crowdworldcontrol(mdp_data, x, u);
[r, g, drdu, d2rdudu, drdx, d2rdxdx] =...
    obstaclesumevalreward(reward, mdp_data, x, u, states, A, B);
d2rdxdx_new = permute(d2rdxdx, [2, 3, 1]);  % for observing the diagonal structure
drdx_fd = zeros(size(drdx));
d2rdxdx_fd = zeros(size(d2rdxdx));
for j = 1:size(drdx, 2)
	dX = zeros(size(states));
	dX(:, j) = dx;
	[r_m, ~, ~, ~, drdx_m] = obstaclesumevalreward(...
        reward, mdp_data, x, u, states-dX, A, B);
	[r_p, ~, ~, ~, drdx_p] = obstaclesumevalreward(...
        reward, mdp_data, x, u, states+dX, A, B);
	drdx_fd(:, j) = (r_p - r_m)/2/dx;
	d2rdxdx_fd(:, :, j) = (drdx_p - drdx_m)/2/dx;
end
drdx_ae = abs_error(drdx, drdx_fd);
drdx_re = rel_error(drdx, drdx_fd);
d2rdxdx_ae = abs_error(d2rdxdx, d2rdxdx_fd);
d2rdxdx_re = rel_error(d2rdxdx, d2rdxdx_fd);
disp('drdx - absolute error');
disp(max(drdx_ae, [], 'all'));
disp('drdx - relative error');
disp(max(drdx_re, [], 'all'));
% disp('drdx - mse error');
% disp(mse_error(drdx, drdx_fd));
disp('d2rdxdx - absolute error');
disp(max(d2rdxdx_ae, [], 'all'));
disp('d2rdxdx - relative error')
disp(max(d2rdxdx_re, [], 'all'));
% disp('d2rdxdx - mse error')
% disp(mse_error(d2rdxdx, d2rdxdx_fd));

%% visualizing
% addpath ./Visualization;
% fprintf('\nSample    ');
% for i = 1:length(samples)
% 	fprintf('\b\b\b');
% 	fprintf('%3i\n', i);
% 	fig = playsample(samples{i}, mdp_data_arr{i}, false, ...
% 		             [0, 60], [-10, 10], walls);
% 	if isgraphics(fig)
% 		close(fig);
% 	else
% 		break;
%     end
% end
% fprintf('\n');
% 

%EOF