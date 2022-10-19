% Construct the robot arm MDP structures.
function [mdp_data,reward,features_pt,features_dyn] = robotarmbuild(mdp_params)

% Fill in default parameters.
mdp_params = robotarmdefaultparams(mdp_params);

% Set random seed.
rng(mdp_params.seed);

% Compute state and action bounds.
sbounds = [-ones(1,mdp_params.links)*pi -ones(1,mdp_params.links)*0.1; ones(1,mdp_params.links)*pi ones(1,mdp_params.links)*0.1];
abounds = [-ones(1,mdp_params.links)*1; ones(1,mdp_params.links)*1];

objects = struct('pos',[],'c1',[],'c2',[]);
% Place objects in a fixed pattern.
objects(1) = struct('pos',[0.5 0.5]*mdp_params.size,'c1',1,'c2',1);
objects(2) = struct('pos',[0.3 0.3]*mdp_params.size,'c1',2,'c2',1);
objects(3) = struct('pos',[0.7 0.3]*mdp_params.size,'c1',2,'c2',1);
objects(4) = struct('pos',[0.3 0.7]*mdp_params.size,'c1',2,'c2',1);
objects(5) = struct('pos',[0.7 0.7]*mdp_params.size,'c1',2,'c2',1);

% Create MDP data structure.
mdp_data = struct(...
    'mdp','robotarm',...
    'dims',mdp_params.links*2,...
    'udims',mdp_params.links,...
    'sbounds',sbounds,'abounds',abounds,...
    'bounds',mdp_params.size*ones(1,2),...
    'links',mdp_params.links,...
    'linklen',mdp_params.linklen,...
    'linkmass',mdp_params.linkmass,...
    'objects',objects);

% Construct feature map.
[reward,features_pt,features_dyn] = robotarmfeatures(mdp_params,mdp_data);
