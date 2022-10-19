% Evaluate statistics on examples and synthesized trajectories.
function results = highwayevalexamples(examples,mdp_data,mdp_params,mdp)

% Evaluates the statistics for each example.
% Statistics are:
% mean speed
% number of collisions
% number of steps behind a car
% number of steps in front of a car

% Allocate memory.
results = zeros(4,length(examples));

% Prepare.
evalstruct = highwayevalprepare(mdp_params);

% Evaluate each example.
for i=1:length(examples),
    % Evaluate control.
    T = size(examples{i}.u,1);
    if T ~= evalstruct.T,
        evalstruct.T = T;
    end;
    x = feval(strcat(mdp,'control'),mdp_data,examples{i}.s,examples{i}.u);
    
    % Step over each time step.
    for t=1:T,
        % Add to statistics.
        results(:,i) = results(:,i) + highwayevalcompute(evalstruct,x(t,:),t,mdp_data);
    end;
end;
