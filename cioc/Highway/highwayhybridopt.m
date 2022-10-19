% Hybrid highway optimization that optimizes over both states and actions.
function [u,r,x,U,X] = highwayhybridopt(x0,u,mdp_data,mdp,reward)

T = size(u,1);
Du = size(u,2);
Dx = mdp_data.dims;

% Set up optimization options.
options = struct();
options.Method = 'lbfgs';
options.maxIter = 1000;
options.MaxFunEvals = 1000;
options.display = 'on';
options.TolX = 1.0e-16;
options.TolFun = 1.0e-8;
options.Display = 'off';
%options.DerivativeCheck = 'on';

% Compute states.
x = feval(strcat(mdp,'control'),mdp_data,x0,u);
x(:,6) = [];
xu = vertcat(u(:),x(:));

FRAC = 1.0;
mu = 1.0*ones(T,5);
lambda = zeros(T,5);
oldp = zeros(T,5);
dp = [0.0 0.0 0.05 0.1 0.15 0.2 0.25 0.4 0.8 1.0];
U = {};
X = {};
for i=1:20,
    % Adjust reward.
    curreward = reward;
    penwt = dp(min(length(dp),i));
    %penwt = 1.0;
    for j=1:length(curreward.features),
        if strcmp(curreward.features{j}.type,'dist'),
            curreward.theta(j) = curreward.theta(j)*penwt;
        end;
    end;
    
    % Run minFunc.
    [xu,~] = minFunc(@(p)highwayhybridcost(p,x0,mdp_data,mdp,curreward,mu,lambda),xu,options);

    % Pull out controls.
    u = reshape(xu(1:numel(u)),T,Du);
    x = [reshape(xu((Du*T+1):end),T,Dx-1) (1:T)'];
    U{i} = u;
    X{i} = x;

    % Compute states and state Hessian.
    states = feval(strcat(mdp,'control'),mdp_data,x0,u);
    p = states-x;
    p(:,6) = [];
    fprintf(1,'Discrepancy: %f Mu: %f Lambda: %f\n',sum(sum(p.^2)),mean(mean(mu)),mean(mean(lambda)));
    
    % Update lambda and mu.
    %lambda = lambda - mu.*p*FRAC;
    %mu = mu.*((abs(p) > 0.5*abs(oldp))*9 + 1);
    mu = mu*10;
    oldp = p;
end;

% Pull out controls.
u = reshape(xu(1:numel(u)),T,Du);
x = [reshape(xu((Du*T+1):end),T,Dx-1) (1:T)'];

% Compute states and state Hessian.
states = feval(strcat(mdp,'control'),mdp_data,x0,u);
fprintf(1,'Final discrepancy: %f\n',sum(sum((states-x).^2)));

% Run final optimization.
r = trajectoryreward(u(:),x0,mdp_data,mdp,reward);
%[u,r] = minFunc(@(p)trajectoryreward(p,x0,mdp_data,mdp,reward),u(:),options);
