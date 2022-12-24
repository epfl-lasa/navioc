%% initializing 
clc; clear; close all;
addpath ./Utilities;

%% defining functions
h = @(z, s) 1./(1+exp(-s*z));
% hg = @(z, s) s*h(z, s).*(1-h(z, s)); 
hg = @(z, s) s*h(z, s).*h(-z, s); 
hh = @(z, s) s*hg(z, s).*(1-2*h(z, s));

%% defining constants
eps_ = 10^-20;
% eps_ = 0;
s = 10;
expec = 1;
Nt = 3;
Nped = 1;
Nu = 2*Nped;
Nx = 4*Nped;
normalizer = Nped*expec; 
Nobs = 1;

%% building obstacles
obstacle = struct('points', [1, 1; 5, 1]);
obstacle.points = sort_coords(obstacle.points, true);
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
obstacles = cell(1, Nobs);
obstacles{1} = obstacle;

%% generating states
states = zeros(Nt, Nx);
states(1, :) = [3, 4, 2000, 2000];
states(2, :) = [1.1, 4, 1000, 1000];
states(3, :) = [1, 4, 500, 500];

%% calculating the feature value
Nobs = size(obstacles, 2); 
idx = 1:(Nu/2);
px_idx = 2*idx - 1;
py_idx = px_idx + 1;
p_idx = reshape([px_idx; py_idx], 1, []);
px = reshape(states(:, px_idx)', 1, [], Nt);    % dim = 1*Nped*Nt (or 1, Nped, [])
py = reshape(states(:, py_idx)', 1, [], Nt);    % dim = 1*Nped*Nt (or 1, Nped, [])
pos = [px; py]; % dim = 2*Nped*Nt
r = zeros(Nt, 1);
h_val = cell(1, Nobs);
f = cell(1, Nobs);
for j = 1:Nobs 
    normals = repmat(obstacles{j}.normals', 1, 1, Nt);  % dim = m*2*Nt 
    offsets = repmat(obstacles{j}.offsets', 1, 1, Nt);  % dim = m*1*Nt
    h_val{j} = h(-(offsets + pagemtimes(normals, pos)), s);  % dim = m*Nped*Nt
    f{j} = prod(h_val{j}, 1);   % dim = 1*Nped*Nt
    r = r + reshape(sum(f{j}, 2)/normalizer, Nt, []); % dim of sum = 1*1*Nt
end

%% calculating the feature gradient
drdx = zeros(Nt, Nx);
hg_val = cell(1, Nobs);
for j = 1:Nobs
    normals = repmat(obstacles{j}.normals', 1, 1, Nt);  % dim = m*2*Nt 
    offsets = repmat(obstacles{j}.offsets', 1, 1, Nt);  % dim = m*1*Nt
    hg_val{j} = hg(-(offsets + pagemtimes(normals, pos)), s);  % dim = m*Nped*Nt
    grad = hg_val{j} ./ (h_val{j} + eps_);  % dim = m*Nped*Nt
    grad = pagemtimes(pagetranspose(grad), -normals);  % dim = Nped*2*Nt
    grad = pagetranspose(f{j}) .* grad;   % dim = Nped*2*Nt
    grad = reshape(pagetranspose(grad), 1, [], Nt);    % dim = 1*(Nped*2)*Nt
    grad = transpose(squeeze(grad)); % dim = Nt*(Nped*2) (reshape(grad, Nped*2, Nt))
    drdx(:, p_idx) = drdx(:, p_idx) + grad; 
end
drdx(:, p_idx) = drdx(:, p_idx) / normalizer;

%% calculating the feature Hessian
d2rdxdx = zeros(Nt, Nx, Nx);
hh_val = cell(1, Nobs);
for j = 1:Nobs  % obstacle j consists of m sides
    normals = repmat(obstacles{j}.normals', 1, 1, Nt);  % dim = m*2*Nt 
    offsets = repmat(obstacles{j}.offsets', 1, 1, Nt);  % dim = m*1*Nt
    hh_val{j} = hh(-(offsets + pagemtimes(normals, pos)), s);  % dim = m*Nped*Nt
    temp = hh_val{j} ./ (h_val{j} + eps_);  % dim = m*Nped*Nt
    hess = zeros(Nped*2, Nped*2, Nt);
    for k = 1:obstacles{j}.num
        normal = pagetranspose(normals(k, :, :)) .* normals(k, :, :); % dim = 2*2*Nt
        for i = 1:Nped
            hess([2*i-1, 2*i], [2*i-1, 2*i], :) = hess([2*i-1, 2*i], [2*i-1, 2*i], :) +...
                f{j}(1, i, :) .* (temp(k, i, :) .* normal);
        end
    end
    temp = h_val{j} ./ (hg_val{j} + eps_);  % dim = m*Nped*Nt
    grad_prod = prod(hg_val{j}, 1); % dim = 1*Nped*Nt
    ids = 1:obstacles{j}.num;
    if length(ids) > 1
        pairs = nchoosek(ids, 2);
        for k = 1:size(pairs, 1)
            ids_c = ids(~ismember(ids, pairs(k, :)));
            temp_k = prod(temp(ids_c, :, :), 1);    % dim = 1*Nped*Nt
            normal = pagetranspose(normals(pairs(k, 1), :, :)) .* normals(pairs(k, 2), :, :); % dim = 2*2*Nt
            normal = pagetranspose(normal) + normal;    % dim = 2*2*Nt
            for i = 1:Nped
                hess([2*i-1, 2*i], [2*i-1, 2*i], :) = hess([2*i-1, 2*i], [2*i-1, 2*i], :) +...
                    grad_prod(1, i, :) .* (temp_k(1, i, :) .* normal);
            end
        end
    end
    d2rdxdx(:, p_idx, p_idx) = d2rdxdx(:, p_idx, p_idx) + permute(hess, [3, 1, 2]); 
end
d2rdxdx(:, p_idx, p_idx) = d2rdxdx(:, p_idx, p_idx) / normalizer;
d2rdxdx_new = permute(d2rdxdx, [2, 3, 1]);