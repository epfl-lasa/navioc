% Fill in default parameters for the Highway example.
function mdp_params = highwaydefaultparams(mdp_params)

% Create default parameters.
default_params = struct(...
    'seed',0,...
    'optimizes',1,...
    'carfile','none',...
    'lanes',3,...
    'lane_space',0.1,...
    'cars',40,...
    'size',[10 10],...
    'max_time',128,...
    'step_cost',10.0,...
    'speed_features',1,...
    'group_lane_features',0,...
    'group_lanes_max',0,...
    'linear_speed',0,...
    'feature_radius',1.0);

% Set parameters.
mdp_params = filldefaultparams(mdp_params,default_params);
