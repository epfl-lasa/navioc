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
obstacles_num = 9;
obstacles = cell(1, obstacles_num);
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
for j = 1:obstacles_num
    % with sort_coords
    obstacles{j}.points = sort_coords(obstacles{j}.points, true);
    if size(obstacles{j}.points, 2) == 2
        obstacles{j}.num = size(obstacles{j}.points, 2)-1;
    elseif size(obstacles{j}.points, 2) > 2
        obstacles{j}.num = size(obstacles{j}.points, 2); % num is the total number of edges
    end
    % with cnvhull
    % k = convhull(obstacles{j}.points(1, :), obstacles{j}.points(2, :));
    % obstacles{j}.points = obstacles{j}.points(:, k);
    % obstacles{j}.num = size(obstacles{j}.points, 2)-1;  % num is the total number of edges
end
% calculating the normals 
for j = 1:obstacles_num
    obstacles{j}.normals = zeros(2, obstacles{j}.num);
    % with sort_coords
    temp = obstacles{j}.points;
    temp = [temp, temp(:, 1)];
    for k = 2:obstacles{j}.num+1
        direction = temp(:, k) - temp(:, k-1);
        obstacles{j}.normals(:, k-1) = [0, 1; -1, 0] * direction;
    end
    % with cnvhull
    % for k = 2:obstacles{i}.num+1
    %     direction = obstacles{j}.points(:, k) - obstacles{j}.points(:, k-1);
    %     obstacles{j}.normals(:, k-1) = [0, 1; -1, 0] * direction;
    % end
    obstacles{j}.normals = normc(obstacles{j}.normals);
end
% calculating the offsets
for j = 1:obstacles_num
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

%% choosing a particular agent
i = 1;  % do not change (1, 12, 16)
x = samples{i}.s(1, [13:14, 35:36]);
u = samples{i}.u(:, 13:14);
mdp_data = mdp_data_arr{i};
mdp_data.n_ped = 1;
mdp_data.dims = 4;
mdp_data.udims = 2;
mdp_data.v_des = mdp_data.v_des(1, 13:14);
mdp_data.idx_other = 1;
mdp_data.type = 0;
dt = mdp_data.time_step;
Nt = size(samples{i}.states, 1);
states = samples{i}.states(:, [13:14, 35:36]);
% [states, A, B] = crowdworldcontrol(mdp_data, x, u);
states(:, 2) = states(:, 2) + 0.0; 
s = 10;
eps_ = 1e-10;
reward = struct('expec', 1, 's', s, 'eps', eps_, 'fast', 1,...
                'type', 'obstaclessum', 'type_c', false, 'type_w', false,...
                'type_other', false);
[r, g, drdu, d2rdudu, drdx, d2rdxdx] =...
    obstaclesumevalreward(reward, mdp_data, x, u, states);
px = reshape(states(:, 1)', 1, [], Nt);    
py = reshape(states(:, 2)', 1, [], Nt);   
pos = [px; py];
% verifying the feature
dx = 1e-5;  
d2rdxdx_new = permute(d2rdxdx, [2, 3, 1]);  % for observing the diagonal structure
drdx_fd = zeros(size(drdx));
d2rdxdx_fd = zeros(size(d2rdxdx));
r_p = zeros(size(drdx));
r_m = zeros(size(drdx));
r_diff = zeros(size(drdx));
for j = 1:size(drdx, 2)
	dX = zeros(size(states));
	dX(:, j) = dx;
	[r_m(:, j), ~, ~, ~, drdx_m] = obstaclesumevalreward(...
        reward, mdp_data, x, u, states-dX);
	[r_p(:, j), ~, ~, ~, drdx_p] = obstaclesumevalreward(...
        reward, mdp_data, x, u, states+dX);
    r_diff(:, j) = r_p(:, j) - r_m(:, j);
	drdx_fd(:, j) = r_diff(:, j)/2/dx;
	d2rdxdx_fd(:, :, j) = (drdx_p - drdx_m)/2/dx;
end
d2rdxdx_fd_new = permute(d2rdxdx_fd, [2, 3, 1]);
d2rdxdx_diff_new = d2rdxdx_new - d2rdxdx_fd_new;
drdx_ae = abs_error(drdx, drdx_fd);
drdx_re = rel_error(drdx, drdx_fd);
d2rdxdx_ae = abs_error(d2rdxdx, d2rdxdx_fd);
d2rdxdx_re = rel_error(d2rdxdx, d2rdxdx_fd);
disp('drdx - absolute error');
disp(max(drdx_ae, [], 'all'));
disp('drdx - relative error');
disp(max(drdx_re, [], 'all'));
disp('d2rdxdx - absolute error');
disp(max(d2rdxdx_ae, [], 'all'));
disp('d2rdxdx - relative error')
disp(max(d2rdxdx_re, [], 'all'));

%% animation
colors = [0 0.4470 0.7410;
          0.8500 0.3250 0.0980;
          0.9290 0.6940 0.1250;
          0.4940 0.1840 0.5560;
          0.4660 0.6740 0.1880;
          0.3010 0.7450 0.9330;
          0.6350 0.0780 0.1840];
gcf = figure('units', 'normalized', 'outerposition', [0, 0, 1, 1]); % maximizing figure window
% map
subplot(1, 2, 1);
axis equal manual;
hold on;
grid on;
box on;
plot(0, 0, 'k.', 'Markersize', 15);
xline(0, '--');
yline(0, '--');
xlabel('$x\ (m)$', 'Interpreter', 'latex');
ylabel('$y\ (m)$', 'Interpreter', 'latex');
title('$Simulation$', 'Interpreter', 'latex');
set(gca, 'TickLabelInterpreter', 'latex');
lim_x = [-5, 60];
lim_y = [-10, 18];
xlim(lim_x);
ylim(lim_y);
for j = 1:obstacles_num
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
% feature value vs time step
subplot(1, 2, 2);
grid on;
box on;
hold on;
o = plot(0, 0, 'k.', 'Markersize', 15);
o.Annotation.LegendInformation.IconDisplayStyle = 'off';
o = xline(0, '--');
o.Annotation.LegendInformation.IconDisplayStyle = 'off';
o = yline(0, '--');
o.Annotation.LegendInformation.IconDisplayStyle = 'off';
xlabel('$time\ step$', 'Interpreter', 'latex');
title('$feature\ value$', 'Interpreter', 'latex');
set(gca, 'TickLabelInterpreter', 'latex');
xlim([1, Nt]);
ylim([0, 1]);
for i = 1:Nt
    % stopping the program when the figure is closed
    if ishghandle(gcf) == 0
        clc;
        close all;
        break;
    end
    % map
    subplot(1, 2, 1);
    agent = plot(pos(1, 1, i), pos(2, 1, i), '.', 'Markersize', 12,...
                 'Color', colors(1, :)); 
    time_step = text(mean(lim_x)-5, lim_y(2)-2, ['$time\ step\ =\ $', num2str(i)],...
                    'Interpreter', 'latex', 'Fontsize', 12);
    % feature value vs time step
    subplot(1, 2, 2);
    if i > 1
        plot([i-1, i], [r(i-1, 1), r(i, 1)], '-',...
             'color', colors(1, :), 'linewidth', 2);
        legend(string(reward.s), 'Interpreter', 'latex',...
               'AutoUpdate', 'off', 'Location', 'best');
    end
    pause(dt);
    delete(agent);
    delete(time_step);
end

%EOF