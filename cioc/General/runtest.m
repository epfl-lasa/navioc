% Run IRL test with specified algorithm and example.
function test_result = runtest(algorithm,algorithm_params,...
    mdp,mdp_params,test_params)

% test_result - structure that contains results of the test
% algorithm - string specifying the IRL algorithm to use
% algorithm_params - parameters of the specified algorithm
% mdp - string specifying example to test on
% mdp_params - string specifying parameters for example
% test_params - general parameters for the test:
%   training_samples (32) - number of example trajectories to query
%   training_sample_lengths (100) - length of each sample trajectory

% Seed random number generators.
rng(1);

% Set default test parameters.
test_params = setdefaulttestparams(test_params);

% Construct MDP and features.
[mdp_data,reward,features_pt,features_dyn] = feval(strcat(mdp,'build'),mdp_params);

% Get example trajectories.
[example_samples,test_samples] = sampleexamples(mdp_data,mdp,reward,test_params,test_params.verbosity);

% Copy discretization settings.
algorithm_params.grid_cells_state = test_params.cells_state;
algorithm_params.grid_cells_action = test_params.cells_action;
algorithm_params.grid_action_quad = test_params.action_quad;

% Run IRL algorithm.
irl_result = feval(strcat(algorithm,'run'),algorithm_params,mdp,mdp_data,...
    features_pt,features_dyn,example_samples,test_params.verbosity);

% Evaluate IRL result by resynthesizing trajectories.
irl_result.example_samples = example_samples;
irl_result.test_samples = test_samples;
[irl_result.example_samples,irl_result.test_samples] = ...
    resampleexamples(mdp_data,mdp,irl_result.reward,reward,test_params,...
                     irl_result.example_samples,irl_result.test_samples,test_params.verbosity);

% Evaluate metrics.
test_metrics = evaluatemetrics(example_samples,irl_result.example_samples,...
    test_samples,irl_result.test_samples,reward,irl_result.reward,irl_result);

% Return result.
test_result = struct('irl_result',irl_result,'reward',reward,...
    'example_samples',{example_samples},'test_samples',{test_samples},...
    'mdp_data',mdp_data,'mdp_params',mdp_params,'mdp',mdp,...
    'algorithm',algorithm,'test_metrics',test_metrics,...
    'features_pt',{features_pt},'features_dyn',{features_dyn});
