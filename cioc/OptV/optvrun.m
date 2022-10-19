% Dvijotham & Todorov OptV algorithm.
function irl_result = optvrun(algorithm_params,mdp,mdp_data,features_pt,...
    features_dyn,example_samples,verbosity)

rng(1);

% Fill in default parameters.
algorithm_params = optvdefaultparams(algorithm_params);

% Get constants.
N = length(example_samples);
T = size(example_samples{1}.u,1);

% This is a little hack to give OptV the step cost.
if mdp_data.dims == 2,
    step_cost = 40.0;
else
    step_cost = 10.0;
end;

% Compute states corresponding to each example.
src_states = zeros(T*N,mdp_data.dims);
dst_states = zeros(T*N,mdp_data.dims);
controls = zeros(T*N,mdp_data.udims);
for i=1:length(example_samples),
    u = example_samples{i}.u;
    s0 = example_samples{i}.s;
    pts = feval(strcat(mdp,'control'),mdp_data,s0,u);
    dst_states((i-1)*T+(1:T),:) = pts;
    src_states((i-1)*T+(1:T),:) = [s0; pts(1:(T-1),:)];
    controls((i-1)*T+(1:T),:) = u;
end;

% Discretize actions.
% First compute bounds.
abounds = [min(controls,[],1); max(controls,[],1)];
act_vals = buildgrid(abounds,algorithm_params.grid_cells_action,1);

% Adjust number of bases to be no more than a quarter the number of points.
if size(dst_states,1) < algorithm_params.bases*8,
    algorithm_params.bases = floor(size(dst_states,1)/8);
end;

% Initialize value function features by clustering the example states.
% Compute centers destination states.
%gmm = gmdistribution.fit(dst_states,algorithm_params.bases,'Regularize',5.0e-2,'Replicates',10,'Options',statset('MaxIter',500));
%means = gmm.mu; % N x Dx means matrix.
%covars = gmm.Sigma; % Dx x Dx x N covariances matrix.
[~,means] = kmeans(dst_states,algorithm_params.bases,'emptyaction','drop','replicates',10);
% Remove dropped means.
means(isnan(means(:,1)),:) = [];

% Compute new covariances using the method suggested by D & T.
% Get the D nearest Gaussians.
covars = zeros(size(means,2),size(means,2),size(means,1));
d2 = sum(bsxfun(@minus,permute(means,[1 3 2]),permute(means,[3 1 2])).^2,3);
[~,idx] = sort(d2,2);
for i=1:size(covars,3),
    covars(:,:,i) = eye(mdp_data.dims,mdp_data.dims)*5.0e-2;
    K = min(mdp_data.dims*2,size(covars,3)-1);
    for j=1:(K+1),
        md = means(idx(i,j),:) - means(i,:);
        covars(:,:,i) = covars(:,:,i) + md'*md;
    end;
    covars(:,:,i) = 2*covars(:,:,i)/K;
end;

% Invert covariances.
for i=1:size(covars,3),
    covars(:,:,i) = inv(covars(:,:,i));
end;

% Pack the features.
theta = optvpackfeatureparams(means,covars);

% For each source point, assemble:
% Successor point in the data, and its probability.
x = src_states;
xpr = dst_states;

% All other successor points, and their probabilities.
allxpr = zeros(size(x,1),size(act_vals,1),size(x,2));
pallxpr = zeros(size(x,1),size(act_vals,1));
for i=1:size(x,1),
    % Evaluate each action.
    for a=1:size(act_vals,1),
        pt = feval(strcat(mdp,'control'),mdp_data,x(i,:),act_vals(a,:));
        allxpr(i,a,:) = pt(1,:);
        pallxpr(i,a) = exp(-step_cost*sum(act_vals(a,:).^2,2));
    end;
end;

% Normalize probabilities.
pallxpr = bsxfun(@rdivide,pallxpr,sum(pallxpr,2));

% Set up optimization options.
options = struct();
options.Display = 'iter';
options.LS_init = 2;
options.LS = 2;
options.Method = 'lbfgs';
%options.DerivativeCheck = 'on';
if verbosity == 0,
    options.display = 'none';
end;

% Initialize weights.
global w;
w = rand(size(covars,3),1);

% Begin timing.
tic;

% Optimize weights.
% Convert theta.
F = size(covars,3);
D = size(allxpr,3);
TPF = 1 + D + 0.5*D*(D+1);
theta3 = permute(reshape(theta,TPF,F),[3 4 2 1]);

% Precompute f(xpr) for xpr.
xprf = exp(sum(bsxfun(@times,optvs(permute(xpr,[1 3 2])),theta3),4));
xprf = min(xprf,1.0e100);
xprf = bsxfun(@rdivide,xprf,max(sum(xprf,3),1.0e-20));

% Precompute f(allxpr) for allxpr.
allxprf = exp(sum(bsxfun(@times,optvs(allxpr),theta3),4));
allxprf = min(allxprf,1.0e100);
allxprf = bsxfun(@rdivide,allxprf,max(sum(allxprf,3),1.0e-20));

% Optimize weights.
w = minFunc(@(w)optvcost(w,pallxpr,xprf,allxprf),w,options);

% Optimize features.
options.maxIter = algorithm_params.basis_iters;
options.MaxFunEvals = algorithm_params.basis_iters;
theta = minFunc(@(t)optvfcost(t,pallxpr,allxpr,xpr,size(covars,3)),theta,options);

% Optimize weights.
% Convert theta.
F = size(covars,3);
D = size(allxpr,3);
TPF = 1 + D + 0.5*D*(D+1);
theta3 = permute(reshape(theta,TPF,F),[3 4 2 1]);

% Precompute f(xpr) for xpr.
xprf = exp(sum(bsxfun(@times,optvs(permute(xpr,[1 3 2])),theta3),4));
xprf = min(xprf,1.0e100);
xprf = bsxfun(@rdivide,xprf,max(sum(xprf,3),1.0e-20));

% Precompute f(allxpr) for allxpr.
allxprf = exp(sum(bsxfun(@times,optvs(allxpr),theta3),4));
allxprf = min(allxprf,1.0e100);
allxprf = bsxfun(@rdivide,allxprf,max(sum(allxprf,3),1.0e-20));

% Optimize weights.
w = minFunc(@(w)optvcost(w,pallxpr,xprf,allxprf),w,options);

% Stop timing.
total_time = toc;

% Create reward.
reward = struct('type','optv','w',w,'theta',theta,'F',F,'act_vals',act_vals);

% Return the reward.
irl_result = struct('reward',reward,'total_time',total_time);
