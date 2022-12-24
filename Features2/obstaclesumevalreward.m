function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	obstaclesumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)
% extracting required info from mdp_data
Nt = size(states, 1);
Nu = mdp_data.udims;
Nx = mdp_data.dims;
Nped = mdp_data.n_ped;
obstacles = mdp_data.obstacles;
Nobs = size(obstacles, 2); 
% fast is false in IRL
fast = isfield(reward, 'fast') && reward.fast;
% the sum will be averaged by 1/(reward.expec*Nped)
normalizer = Nped*reward.expec;  
% hyperparameter of the sigmoid
s = reward.s;
% activation function
h = @(z, s) 1./(1+exp(-s*z));
% gradient of h(z) is s*h(z)*(1-h(z)) and (1-h(z)) is h(-z)
hg = @(z, s) s*h(z, s).*(1-h(z, s));
% hg = @(z, s) s*h(z, s).*h(-z, s);   % more numerically stable than above
% hessian of h(z) is (s^2)*h(z)*(1-h(z))*(1-2h(z)) = s*hg(z)*(1-2h(z))
hh = @(z, s) s*hg(z, s).*(1-2*h(z, s));
% epsilon for numerical stability
eps_ = reward.eps;
% copied from verr2sumevalreward
if reward.type_other
    idx = mdp_data.idx_other;
elseif reward.type_w
    idx = mdp_data.idx_wheelchair;
elseif reward.type_c
    idx = mdp_data.idx_companion;
else
%     idx = 1:(Nu/2); % it effectively is 1:n_ped
    idx = 1:Nped;
end
% extracting the pos_x (px) and pos_y (py) indices in states
px_idx = 2*idx - 1;
py_idx = px_idx + 1;
p_idx = reshape([px_idx; py_idx], 1, []);
% building pos array
px = reshape(states(:, px_idx)', 1, [], Nt);    % dim = 1*Nped*Nt (or 1, Nped, [])
py = reshape(states(:, py_idx)', 1, [], Nt);    % dim = 1*Nped*Nt (or 1, Nped, [])
pos = [px; py]; % dim = 2*Nped*Nt
% calculating the feature value
r = zeros(Nt, 1);
h_val = cell(1, Nobs);
f = cell(1, Nobs);
normals = cell(1, Nobs);
offsets = cell(1, Nobs);
z = cell(1, Nobs);
for j = 1:Nobs  % obstacle j consists of m_j (m to be concise) sides
    normals{j} = repmat(obstacles{j}.normals', 1, 1, Nt);   % dim = m*2*Nt 
    offsets{j} = repmat(obstacles{j}.offsets', 1, 1, Nt);   % dim = m*1*Nt
    z{j} = -(offsets{j} + pagemtimes(normals{j}, pos));     % dim = m*Nped*Nt
    h_val{j} = h(z{j}, s);                                  % dim = m*Nped*Nt
    h_val{j}(abs(h_val{j}) < eps_) = 0;                     % dim = m*Nped*Nt
    % f(j) consists of multiplication of activation associated with all 
    % agents and obstacle j over the horizon
    f{j} = prod(h_val{j}, 1);               % dim = 1*Nped*Nt
    f{j}(abs(f{j}) < eps_) = 0;             % dim = 1*Nped*Nt
    r = r + reshape(sum(f{j}, 2), Nt, []);  % dim of sum = 1*1*Nt
end
r = r /normalizer;      % dim = Nt*1
r(abs(r) < eps_) = 0;   % dim = Nt*1
if nargout >= 2
	drdx = zeros(Nt, Nx);
    hg_val = cell(1, Nobs);
    for j = 1:Nobs  % obstacle j consists of m_j (m to be concise) sides
        hg_val{j} = hg(z{j}, s);  % dim = m*Nped*Nt
        hg_val{j}(abs(hg_val{j}) < eps_) = 0;   % dim = m*Nped*Nt
        grad = zeros(2, Nped, Nt);
        ids = 1:obstacles{j}.num;
        for k = ids
            ids_c = ids(~ismember(ids, k));
            if length(ids) > 1
                temp = prod(h_val{j}(ids_c, :, :), 1);    % dim = 1*Nped*Nt
                temp = temp .* hg_val{j}(k, :, :);  % dim = 1*Nped*Nt
            else
                temp = hg_val{j}(k, :, :);  % dim = 1*Nped*Nt
            end
            temp(abs(temp) < eps_) = 0; % dim = 1*Nped*Nt
            temp = pagemtimes(pagetranspose(-normals{j}(k, :, :)), temp); % dim = 2*Nped*Nt
            grad = grad + temp; % dim = 2*Nped*Nt
        end
        grad = reshape(grad, 1, [], Nt);    % dim = 1*(Nped*2)*Nt, pagetranspose(grad)
        grad = transpose(squeeze(grad)); % dim = Nt*(Nped*2) (reshape(grad, Nped*2, Nt))
        drdx(:, p_idx) = drdx(:, p_idx) + grad; 
    end
% 	drdx(:, p_idx) = drdx(:, p_idx) / normalizer;
    drdx = drdx / normalizer;
    drdx(abs(drdx) < eps_) = 0;
    % code for g is copied from verr2sumevalreward
	if fast
		g = 0;
	else
		g = permute(gradprod(A, B, permute(drdx, [1, 3, 2])), [1, 3, 2]);
	end
end
% this feature is state-dependent not action-dependent; therefore, dr_du
% and d2r_du2 are 0 (gradients and Hessians of feature w.r.t. action)
if nargout >= 3
	if fast
		drdu = 0;
		d2rdudu = 0;
	else
		drdu = zeros(Nt, Nu);
		d2rdudu = zeros(Nt, Nu, Nu);
	end
end
if nargout >= 6
	d2rdxdx = zeros(Nt, Nx, Nx);
    hh_val = cell(1, Nobs);
    for j = 1:Nobs  % obstacle j consists of m sides
        hh_val{j} = hh(z{j}, s);  % dim = m*Nped*Nt
        hh_val{j}(abs(hh_val{j}) < eps_) = 0;
        hess = zeros(Nped*2, Nped*2, Nt);
        ids = 1:obstacles{j}.num;
        for k = ids
            ids_c = ids(~ismember(ids, k));
            if length(ids) > 1
                temp = prod(h_val{j}(ids_c, :, :), 1);    % dim = 1*Nped*Nt
                temp = temp .* hh_val{j}(k, :, :);  % dim = 1*Nped*Nt
            else
                temp = hh_val{j}(k, :, :);  % dim = 1*Nped*Nt
            end
            temp(abs(temp) < eps_) = 0;
            normal = pagemtimes(pagetranspose(normals{j}(k, :, :)), normals{j}(k, :, :)); % dim = 2*2*Nt
            for i = 1:Nped
                hess([2*i-1, 2*i], [2*i-1, 2*i], :) = hess([2*i-1, 2*i], [2*i-1, 2*i], :) +...
                    pagemtimes(temp(1, i, :), normal);
            end
        end
        if length(ids) > 1
            pairs = nchoosek(ids, 2);
            for k = 1:size(pairs, 1)
                ids_c = ids(~ismember(ids, pairs(k, :)));
                temp = prod(h_val{j}(ids_c, :, :), 1);    % dim = 1*Nped*Nt
                temp = temp .* prod(hg_val{j}(pairs(k, :), :, :), 1);    % dim = 1*Nped*Nt
                temp(abs(temp) < eps_) = 0;
                normal = pagemtimes(pagetranspose(normals{j}(pairs(k, 1), :, :)),...
                                    normals{j}(pairs(k, 2), :, :)); % dim = 2*2*Nt
                normal = normal + pagetranspose(normal);    % dim = 2*2*Nt
                for i = 1:Nped
                    hess([2*i-1, 2*i], [2*i-1, 2*i], :) = hess([2*i-1, 2*i], [2*i-1, 2*i], :) +...
                        pagemtimes(temp(1, i, :), normal);
                end
            end
        end
        d2rdxdx(:, p_idx, p_idx) = d2rdxdx(:, p_idx, p_idx) + permute(hess, [3, 1, 2]); 
    end
% 	d2rdxdx(:, p_idx, p_idx) = d2rdxdx(:, p_idx, p_idx) / normalizer;
    d2rdxdx = d2rdxdx / normalizer;
    d2rdxdx(abs(d2rdxdx) < eps_) = 0;
end

% part of the drdx calculation where we used the factorized reformulation
% grad = hg_val{j} ./ (h_val{j} + eps_);  % dim = m*Nped*Nt
% grad = pagemtimes(pagetranspose(grad), -normals{j});  % dim = Nped*2*Nt
% grad = pagetranspose(f{j}) .* grad;   % dim = Nped*2*Nt

% d2rdxdx calculation where we used the factorized reformulation
% temp = hh_val{j} ./ (h_val{j} + eps_);  % dim = m*Nped*Nt
% hess = zeros(Nped*2, Nped*2, Nt);
% for k = 1:obstacles{j}.num
%     normal = pagemtimes(pagetranspose(normals{j}(k, :, :)), normals{j}(k, :, :)); % dim = 2*2*Nt
%     for i = 1:Nped
%         hess([2*i-1, 2*i], [2*i-1, 2*i], :) = hess([2*i-1, 2*i], [2*i-1, 2*i], :) +...
%             f{j}(1, i, :) .* (temp(k, i, :) .* normal);
%     end
% end
% temp = h_val{j} ./ (hg_val{j} + eps_);  % dim = m*Nped*Nt
% grad_prod = prod(hg_val{j}, 1); % dim = 1*Nped*Nt
% ids = 1:obstacles{j}.num;
% if length(ids) > 1
%     pairs = nchoosek(ids, 2);
%     for k = 1:size(pairs, 1)
%         ids_c = ids(~ismember(ids, pairs(k, :)));
%         temp_k = prod(temp(ids_c, :, :), 1);    % dim = 1*Nped*Nt
%         normal = pagemtimes(pagetranspose(normals{j}(pairs(k, 1), :, :)),...
%                             normals{j}(pairs(k, 2), :, :); % dim = 2*2*Nt
%         normal = normal + pagetranspose(normal);    % dim = 2*2*Nt
%         for i = 1:Nped
%             hess([2*i-1, 2*i], [2*i-1, 2*i], :) = hess([2*i-1, 2*i], [2*i-1, 2*i], :) +...
%                 grad_prod(1, i, :) .* (temp_k(1, i, :) .* normal);
%         end
%     end
% end