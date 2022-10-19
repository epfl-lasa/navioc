% Cost and gradient of hybrid optimization objective.
function [val,grad] = highwayhybridcost(xu,x0,mdp_data,mdp,reward,mu,lambda)

% Constants.
Dx = mdp_data.dims;
Du = mdp_data.udims;
T = numel(xu)/(Dx-1+Du);

% Get u and x.
u = reshape(xu(1:(Du*T)),T,Du);
x = [reshape(xu((Du*T+1):end),T,Dx-1) (1:T)'];

% Compute states and state Hessian.
[states,A,B] = feval(strcat(mdp,'control'),mdp_data,x0,u);

% Determine gradient of x.
[r,~,drdu,~,drdx] = feval(strcat(reward.type,'evalreward'),reward,mdp_data,x0,u,x,A,B,[]);

% Determine discrepancy penalty.
mu = [mu zeros(T,1)];
lambda = [lambda zeros(T,1)];
p = sum(0.5*mu.*((x-states).^2),2);
lamp = sum(lambda.*(x-states),2);
dpdxmu = mu.*(x-states);
dpdxmu(:,6) = [];
dpdxlam = lambda(:,1:5);
drdx(:,6) = [];
dpdumu = -permute(gradprod(A,B,permute((x-states).*mu,[1 3 2])),[1 3 2]);
dpdulam = -permute(gradprod(A,B,permute(lambda,[1 3 2])),[1 3 2]);

% Compute final values and gradients.
val = sum(-r) + sum(p) - sum(lamp);
grad = vertcat(-drdu(:) + dpdumu(:) - dpdulam(:),-drdx(:) + dpdxmu(:) - dpdxlam(:));
