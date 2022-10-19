% Sample example tranjectories from the state space of a given MDP.
function [example_samples,test_samples] = sampleexamples(mdp_data,mdp,reward,test_params,verbose)

% Allocate training samples.
N = test_params.training_samples;
Nt = test_params.test_samples;
T = test_params.training_sample_lengths;
example_samples = cell(1,N);
test_samples = cell(1,Nt);

% Compute and solve discrete MDP for bootstrapping continuous optimization.
if test_params.example_optimal || test_params.test_optimal,
    discrete_mdp = discretesolve(T,mdp_data,mdp,reward,test_params);
else
    discrete_mdp = [];
end;

% Sample test trajectories.
if isfield(test_params,'user_tests'),
    test_samples = test_params.user_tests;
else
    for i=1:Nt,
        if verbose > 0,
            fprintf(1,'Preparing test trajectory %i of %i\n',i,Nt);
        end;

        % Sample initial state.
        s = feval(strcat(mdp,'samplestates'),1,mdp_data);

        % Run optimization controller.
        [states,u,initu,r] = optimizetrajectory(s,T,mdp_data,discrete_mdp,mdp,reward,reward,test_params.test_optimal,test_params.test_restarts);

        % Create example struct.
        test_samples{i} = struct('s',s,'u',u,'initu',initu,'states',states,'r',r);
    end;
end;

% Sample example trajectories.
if isfield(test_params,'user_examples'),
    example_samples = test_params.user_examples;
else
    for i=1:N,
        if verbose > 0,
            fprintf(1,'Preparing example %i of %i\n',i,N);
        end;

        % Sample initial state.
        s = feval(strcat(mdp,'samplestates'),1,mdp_data);

        % Run optimization controller.
        [states,u,initu,r] = optimizetrajectory(s,T,mdp_data,discrete_mdp,mdp,reward,reward,test_params.example_optimal,test_params.example_restarts);

        % Create example struct.
        example_samples{i} = struct('s',s,'u',u,'initu',initu,'states',states,'r',r);
    end;
end;
