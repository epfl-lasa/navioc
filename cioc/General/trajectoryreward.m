% Return the reward obtained along trajectory determined by given inputs,
% as well as the gradient of the reward with respect to those inputs.
function [val,grad] = trajectoryreward(u,s,mdp_data,mdp,reward)

% Reshape.
u = reshape(u,size(u,1)/mdp_data.udims,mdp_data.udims);

% Compute states and state Hessian.
[states,A,B] = feval(strcat(mdp,'control'),mdp_data,s,u);

% Compute reward and its gradient.
[val,drdu] = feval(strcat(reward.type,'evalreward'),reward,mdp_data,s,u,states,A,B,[]);

% Compute gradient of reward with respect to control.
grad = -drdu(:);
val = -sum(val);
