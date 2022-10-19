% Special trajectory optimization routine for OptV.
function [u,r] = optvoptimizetrajectory(u,s,mdp_data,mdp,reward,options)

% In this mode, we optimize one control at a time to take the most likely
% step each time.
T = size(u,1);
Dx = size(s,2);
Du = size(u,2);
curpt = s;
r = 0;
for t=1:T,
    % Optimize the control for this state.
    [u(t,:),cr] = minFunc(@(p)optvtrajectorycost(p,curpt,mdp_data,mdp,reward),u(t,:)',options);
    % Add to total value.
    r = r + cr;
    % Step to next point.
    curpt = feval(strcat(mdp,'control'),mdp_data,curpt,u(t,:));
end;
