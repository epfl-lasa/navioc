% Evaluate the negative likelihood of an OptV step and its gradients.
function [val,grad] = optvtrajectorycost(u,s,mdp_data,mdp,reward)

% Prepare to run.
u = u';
Dx = size(s,2);
Du = size(u,2);

% Evaluate probability score.
logpxpr = -0.5*sum(u.^2,2);
glogpxpr = -u;

% Compute the next state.
[x,~,~,~,dxdu] = feval(strcat(mdp_data.mdp,'control'),mdp_data,s,u);

% Convert theta.
F = reward.F;
TPF = 1 + Dx + 0.5*Dx*(Dx+1);
theta3 = permute(reshape(reward.theta,TPF,F),[3 4 2 1]);

% Precompute f(xpr) for xpr.
% This is N x 1 x F matrix
xf = exp(sum(bsxfun(@times,optvs(permute(x,[1 3 2])),theta3),4));
xf = min(xf,1.0e20);
% This is N x 1 x F x D matrix
xfg = permute(bsxfun(@times,xf,sum(bsxfun(@times,optvs(permute(x,[1 3 2]),1),theta3),4)),[1 2 3 5 4]);
% Now apply normalization.
xfn = max(sum(xf,3),1.0e-20);
xfg = bsxfun(@minus,bsxfun(@rdivide,xfg,xfn),bsxfun(@times,bsxfun(@rdivide,xf,xfn.^2),sum(xfg,3)));
xf = bsxfun(@rdivide,xf,xfn);

% Precompute w3.
w3 = permute(reward.w,[2 3 1]);

% Evaluate the value.
num = sum(bsxfun(@times,xf,w3),3);

if nargout > 1,
    % Evaluate numerator gradient.
    dnumdx = permute(sum(bsxfun(@times,xfg,w3),3),[1 4 2 3]); % 1 x Dx array.
    grad = dxdu * dnumdx';
    grad = - grad - glogpxpr';
end;

val = - num - logpxpr;

