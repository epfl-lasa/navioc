%% initializing 
clc; clear; close all;

%% loading data
load('./Data/diamor_1_s1e-2.mat');
samples = samples_data.samples;
mdp_data_arr = samples_data.mdp_data_arr;

%% choosing a particular agent and generating an arbitrary obstacle
% selecting the agent
i = 2;  % do not change
mdp_data = mdp_data_arr{i};
dt = mdp_data.time_step;
Nt = size(samples{i}.states, 1);
states = samples{i}.states(:, 7:8);
px = reshape(states(:, 1)', 1, [], Nt); 
py = reshape(states(:, 2)', 1, [], Nt);   
pos = [px; py];
% building the obstacle
obstacle = struct('points', [22, 24, 24, 22; 0, 0, 5, 5]);
k = convhull(obstacle.points(1, :), obstacle.points(2, :));
obstacle.points = obstacle.points(:, k);
obstacle.num = size(obstacle.points, 2)-1;
obstacle.normals = zeros(2, obstacle.num);
for j = 2:obstacle.num+1
    direction = obstacle.points(:, j) - obstacle.points(:, j-1);
    obstacle.normals(:, j-1) = [0, 1; -1, 0] * direction;
end
obstacle.normals = normc(obstacle.normals);
obstacle.offsets = zeros(1, obstacle.num);
for j = 1:obstacle.num
    obstacle.offsets(j) = -obstacle.normals(:, j)' * obstacle.points(:, j);
end

%% calculating the feature value
normals = repmat(obstacle.normals', 1, 1, Nt); 
offsets = repmat(obstacle.offsets', 1, 1, Nt);  
temp = -(offsets + pagemtimes(normals, pos));
normalizer = 1; 
s_arr = [1, 2, 5, 10, 50];  % a set of arbitrary values
r = zeros(Nt, size(s_arr, 2));
h = @(z, s) 1./(1+exp(-s*z));
for i = 1:size(s_arr, 2)
    activation = h(temp, s_arr(i));  
    f = prod(activation, 1);  
    r(:, i) = r(:, i) + reshape(sum(f, 2)/normalizer, Nt, []);
end

%% animation
colors = [0 0.4470 0.7410;
          0.8500 0.3250 0.0980;
          0.9290 0.6940 0.1250;
          0.4940 0.1840 0.5560;
          0.4660 0.6740 0.1880;
          0.3010 0.7450 0.9330;
          0.6350 0.0780 0.1840];
lim_x = [19, 26];
lim_y = [-3, 8];
gcf = figure('units', 'normalized', 'outerposition', [0, 0, 1, 1]); % maximizing figure window
% gcf = figure;
% map
% subplot(1, 3, 1);
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
xlim(lim_x);
ylim(lim_y);
plot(obstacle.points(1, :), obstacle.points(2, :), 'k-');
for j = 1:obstacle.num
    midpoint = (obstacle.points(:, j)+obstacle.points(:, j+1))/2; 
    normal = obstacle.normals(:, j);
    quiver(midpoint(1), midpoint(2), normal(1), normal(2), 0,...
           'linewidth', 1, 'color', 'blue');
end
% feature value vs x position
% subplot(1, 3, 2);
subplot(1, 2, 2);
hold on;
grid on;
box on;
o = plot(0, 0, 'k.', 'Markersize', 15);
o.Annotation.LegendInformation.IconDisplayStyle = 'off';
o = xline(0, '--');
o.Annotation.LegendInformation.IconDisplayStyle = 'off';
o = yline(0, '--');
o.Annotation.LegendInformation.IconDisplayStyle = 'off';
xlabel('$x (m)$', 'Interpreter', 'latex');
title('$feature\ value$', 'Interpreter', 'latex');
set(gca, 'TickLabelInterpreter', 'latex');
xlim(lim_x);
ylim([0, 1]);
% feature value vs time step
% subplot(1, 3, 3);
% grid on;
% box on;
% hold on;
% o = plot(0, 0, 'k.', 'Markersize', 15);
% o.Annotation.LegendInformation.IconDisplayStyle = 'off';
% o = xline(0, '--');
% o.Annotation.LegendInformation.IconDisplayStyle = 'off';
% o = yline(0, '--');
% o.Annotation.LegendInformation.IconDisplayStyle = 'off';
% xlabel('$time\ step$', 'Interpreter', 'latex');
% title('$feature\ value$', 'Interpreter', 'latex');
% set(gca, 'TickLabelInterpreter', 'latex');
% xlim([1, Nt]);
% ylim([0, 1]);
for i = 1:Nt
    % stopping the program when the figure is closed
    if ishghandle(gcf) == 0
        clc;
        close all;
        break;
    end
    % map
%     subplot(1, 3, 1);
    subplot(1, 2, 1);
    agent = plot(pos(1, 1, i), pos(2, 1, i), '.', 'Markersize', 18, 'Color', 'red'); 
    if i == 1
        plot(pos(1, 1, i), pos(2, 1, i), 'x', 'Markersize', 12, 'Color', 'black'); 
        text(pos(1, 1, i)-0.3, pos(2, 1, i)-0.3, '$start$',...
             'Interpreter', 'latex', 'Fontsize', 16);
    end
    if i > 1
        plot([pos(1, 1, i-1), pos(1, 1, i)],...
             [pos(2, 1, i-1), pos(2, 1, i)], '-.', 'LineWidth', 1, 'Color', colors(3, :)); 
    end
    time_step = text(mean(lim_x)-3, lim_y(end)-1, ['$time\ step\ =\ $', num2str(i)],...
                    'Interpreter', 'latex', 'Fontsize', 16);
    % feature value vs x position
%     subplot(1, 3, 2);
    subplot(1, 2, 2);
    if i > 1
        for j = 1:size(r, 2)
            plot([pos(1, 1, i-1), pos(1, 1, i)], [r(i-1, j), r(i, j)], '-',...
                 'color', colors(j, :), 'linewidth', 2);
        end
        legend('$s =\ $'+string(s_arr), 'Interpreter', 'latex', 'AutoUpdate','off');
    end
    % feature value vs time step
%     subplot(1, 3, 3);
%     if i > 1
%         for j = 1:size(r, 2)
%             plot([i-1, i], [r(i-1, j), r(i, j)], '-',...
%                  'color', colors(j, :), 'linewidth', 2);
%         end
%         legend(string(s_arr), 'Interpreter', 'latex', 'AutoUpdate','off');
%     end
    if i < Nt
        pause(dt);
        delete(agent);
        delete(time_step);
    end
    if i == Nt
        subplot(1, 2, 1);
        plot(pos(1, 1, i), pos(2, 1, i), 'x', 'Markersize', 12, 'Color', 'black'); 
        text(pos(1, 1, i)-0.3, pos(2, 1, i)-0.3, '$end$',...
             'Interpreter', 'latex', 'Fontsize', 16);
    end
end

%% heatmap
% calculating feature value over a mesh grid
lim_x = [16, 30];
lim_y = [-5, 10];
Ns = 100;
px = linspace(lim_x(1), lim_x(2), Ns);
py = linspace(lim_y(1), lim_y(2), Ns);
[X, Y] = meshgrid(px, py);
pos = X;
pos(:, :, 2) = Y;
pos = permute(pos, [3, 1, 2]);
temp = zeros(obstacle.num, Ns, Ns);
for i=1:Ns
    for j=1:Ns
        temp(:, i, j) = -(obstacle.offsets' + obstacle.normals' * pos(:, i, j));
    end
end
normalizer = 1; 
s_arr = [1, 2, 5, 10];
r = zeros(size(s_arr, 2), Ns, Ns);
h = @(z, s) 1./(1+exp(-s*z));
for i = 1:size(s_arr, 2)
    activation = h(temp, s_arr(i));  
    f = prod(activation, 1);  
    r(i, :, :) = f/normalizer;
end
% plotting
gcf = figure('units', 'normalized', 'outerposition', [0, 0, 1, 1]); % maximizing figure window
m = floor((size(s_arr, 2)+1)/2);
for i = 1:size(s_arr, 2)
    subplot(2, m, i);
    grid on;
    box on;
    h = heatmap(squeeze(r(i, :, :)));
    xlabels = 1:Ns;
    xlabels = string(xlabels);
    xlabels(:) = '';
    h.XDisplayLabels = xlabels;
    xlabel('x');
    ylabels = 1:Ns;
    ylabels = string(ylabels);
    ylabels(:) = '';
    h.YDisplayLabels = ylabels;
    ylabel('y');
    title('s = '+string(s_arr(i)));
end
% print(gcf, './Figures/heatmap.svg', '-dsvg', '-r0');

%% 3d surface
gcf = figure('units', 'normalized', 'outerposition', [0, 0, 1, 1]); % maximizing figure window
m = floor((size(s_arr, 2)+1)/2);
for i = 1:size(s_arr, 2)
    subplot(2, m, i);
    hold on;
    grid on;
    box on;
    view(3);
    surface(X, Y, squeeze(r(i, :, :)));
    colorbar;
    set(gca, 'TickLabelInterpreter', 'latex');
    title('$s = '+string(s_arr(i))+'$', 'Interpreter', 'latex');
end
% print(gcf, './Figures/surface.svg', '-dsvg', '-r0');

%EOF