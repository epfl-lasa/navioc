% Dummy IRL algorithm.
function irl_result = dummyrun(algorithm_params,mdp,mdp_data,features_pt,...
    features_dyn,example_samples,verbosity)

rng(1);

% Concatenate features.
features = [features_dyn features_pt];

% Create sum reward.
reward = struct('type','sum','theta',zeros(1,length(features)),'features',{features});
reward.theta(1) = 1;

% Return the reward.
irl_result = struct('reward',reward,'total_time',1.0);
