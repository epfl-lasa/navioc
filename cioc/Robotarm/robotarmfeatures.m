% Construct the features and reward function for the robot arm domain.
function [reward,features_pt,features_dyn] = robotarmfeatures(mdp_params,mdp_data)

POSITIVE = 10;
NEGATIVE = -15;

% First create the features, which for now are RBF functions centered at
% each object.
features_pt = cell(1,length(mdp_data.objects));
features_dyn = cell(1,1);
features_dyn{1} = struct('type','dist','r',-1.0,'idx',1:mdp_data.udims);
for i=1:length(mdp_data.objects),
    features_pt{i} = struct('type','fkrbf','pos',[mdp_data.objects(i).pos(:,1) mdp_data.objects(i).pos(:,end)],...
                      'width',mdp_params.feature_radius,'r',1.0);
end;

% Create reward as linear combination of features. Objects where c1 = 1 are
% assigned a reward of POSITIVE, objects where c1 = 2 are assigned NEGATIVE.
% First create the weights.
theta = zeros(1,length(mdp_data.objects)+1);
theta(1) = mdp_params.step_cost;
for i=1:length(mdp_data.objects),
    if mdp_data.objects(i).c1 == 1,
        theta(i+1) = POSITIVE;
    elseif mdp_data.objects(i).c1 == 2,
        theta(i+1) = NEGATIVE;
    end;
end;

% Create the reward.
reward = struct('type','sum','theta',theta,'features',{[features_dyn features_pt]});

% Now create seperate features if desired.
if strcmp(mdp_params.feature_type,'simple'),
    % Simple features - simply identity mappings for the states.
    features_pt = cell(1,mdp_data.dims);
    for i=1:mdp_data.dims,
        features_pt{i} = struct('type','id','idx',i,'r',1.0);
    end;
elseif strcmp(mdp_params.feature_type,'cartesian'),
    % Simple features corresponding to Cartesian coordinates.
    features_pt = cell(1,2);
    features_pt{1} = struct('type','fk','idx',1,'r',1.0);
    features_pt{2} = struct('type','fk','idx',2,'r',1.0);
elseif strcmp(mdp_params.feature_type,'grid'),
    % RBF grid.
    GRID_STEPS = 10;
    features_pt = cell(1,GRID_STEPS*GRID_STEPS);
    for x=1:GRID_STEPS,
        for y=1:GRID_STEPS,
            i = (y-1)*GRID_STEPS+x;
            features_pt{i} = struct('type','fkrbf','pos',[x-1 y-1]*mdp_params.size/GRID_STEPS,...
                              'width',mdp_params.feature_radius,'r',1.0);
        end;
    end;
end;
