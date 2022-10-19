% Resample example tranjectories from the state space of a given MDP using a new reward.
function [example_samples,test_samples] = resampleexamples(mdp_data,mdp,reward,true_reward,test_params,old_examples,old_tests,verbose)

% Allocate training samples.
N = length(old_examples);
Nt = length(old_tests);
T = test_params.training_sample_lengths;
example_samples = old_examples;
test_samples = old_tests;

% Compute and solve discrete MDP for bootstrapping continuous optimization.
if test_params.example_recompute_optimal || test_params.test_optimal,
    discrete_mdp = discretesolve(T,mdp_data,mdp,reward,test_params);
else
    discrete_mdp = [];
end;

% Sample test trajectories.
for i=1:Nt,
    if verbose > 0,
        fprintf(1,'Recomputing test trajectory %i of %i\n',i,Nt);
    end;
    if test_params.test_optimal,
        [test_samples{i}.states,test_samples{i}.u,test_samples{i}.initu,test_samples{i}.r] = ...
            optimizetrajectory(test_samples{i}.s,T,mdp_data,discrete_mdp,mdp,reward,true_reward,test_params.test_optimal,test_params.test_restarts);
    else
        [test_samples{i}.states,test_samples{i}.u,test_samples{i}.r] = ...
            reoptimizetrajectory(test_samples{i},mdp_data,mdp,reward,true_reward);
    end;
end;

% Sample example trajectories.
for i=1:N,
    if verbose > 0,
        fprintf(1,'Recomputing example %i of %i\n',i,N);
    end;
    if test_params.example_recompute_optimal,
        [example_samples{i}.states,example_samples{i}.u,example_samples{i}.initu,example_samples{i}.r] = ...
            optimizetrajectory(example_samples{i}.s,T,mdp_data,discrete_mdp,mdp,reward,true_reward,test_params.example_recompute_optimal,test_params.example_restarts);
    else
        [example_samples{i}.states,example_samples{i}.u,example_samples{i}.r] = ...
            reoptimizetrajectory(example_samples{i},mdp_data,mdp,reward,true_reward);
    end;
end;
