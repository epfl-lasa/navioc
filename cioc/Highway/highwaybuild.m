% Construct the Highway MDP structures.
function [mdp_data,reward,features_pt,features_dyn] = highwaybuild(mdp_params)

% The state space is:
% X position
% Y position
% heading
% speed
% angular velocity
% time
% The action space is:
% linear acceleration (gas)
% angular acceleration (steering)

% Fill in default parameters.
mdp_params = highwaydefaultparams(mdp_params);

% Set random seed.
rng(mdp_params.seed);

% Compute state and action bounds.
sbounds = [0, 0, -pi, -0.2, -0.1, 0; mdp_params.size(1), mdp_params.size(2), pi, 0.4, 0.1, mdp_params.max_time];
abounds = [-0.5 -0.5; 0.5 0.5];

% Create MDP data structure.
mdp_data = struct(...
    'mdp','highway',...
    'dims',6,...
    'udims',2,...
    'bounds',mdp_params.size,...
    'sbounds',sbounds,...
    'abounds',abounds,...
    'cardamp',[0.95 0.98],...
    'carmass',[100.0 5.0]);

if mdp_params.optimizes,
    mdp_data.optimizes = 1;
end;

% Initialize cars.
cars = cell(1,mdp_params.cars);

% Set dummy initial configurations.
for i=1:length(cars),
    x0 = zeros(1,6);
    u = highwaysamplecontrols(mdp_params.max_time,mdp_data);
    states = highwaycontrol(mdp_data,x0,u);
    cars{i} = struct('x0',x0,'x',states,'u',u);
end;

% Add cars to data.
mdp_data.cars = cars;

% Construct feature map.
[reward,features_pt,features_dyn] = highwayfeatures(mdp_params,mdp_data);
% Copy track information to data.
mdp_data.xs = features_pt{1}.xs;
mdp_data.xe = features_pt{1}.xe;
mdp_data.ys = features_pt{1}.ys;
mdp_data.ye = features_pt{1}.ye;
mdp_data.cc = features_pt{1}.cc;
mdp_data.cr = features_pt{1}.cr;
mdp_data.cq = features_pt{1}.cq;
% Copy all other lanes.
mdp_data.lanes = features_pt(1:mdp_params.lanes);

ist = length(reward.theta)-length(cars);
if strcmp(mdp_params.carfile,'none'),
    % Create new cars.
    % Set random initial configurations for cars.
    for i=1:length(cars),
        mdp_data.cars{i}.x0 = highwaysamplestates(1,mdp_data);
    end;
    
    % Construct a blank reward function for each car (no other cars).
    car_reward_blank = cell(1,length(cars));
    for i=1:length(cars),
        % Set the reward.
        car_reward_blank{i} = reward;

        % Randomize speed weight.
        car_reward_blank{i}.theta(ist) = car_reward_blank{i}.theta(3)*(0.5+rand(1,1)*0.5);
        
        % Randomize lane affinity.
        ln = randi(mdp_params.lanes);
        car_reward_blank{i}.features{ist-1} = car_reward_blank{i}.features{ist-2-mdp_params.lanes+ln};
        car_reward_blank{i}.features{ist-1}.width = reward.features{ist-1}.width;
        for j=1:mdp_params.lanes,
            if j ~= ln,
                car_reward_blank{i}.theta(ist-2-mdp_params.lanes+j) = 0.25*car_reward_blank{i}.theta(ist-2-mdp_params.lanes+j);
            end;
        end;
        
        % Verify types.
        if ~strcmp(car_reward_blank{i}.features{ist-1}.type,'hwlane') || ...
           ~strcmp(car_reward_blank{i}.features{ist}.type,'id'),
           error('Feature index mismatch!');
        end;

        % Remove the cars.
        car_reward_blank{i}.theta(ist+1:end) = [];
        car_reward_blank{i}.features(ist+1:end) = [];
    end;

    % Construct a reward function for each car.
    car_reward = cell(1,length(cars));
    for i=1:length(cars),
        % Set the reward.
        car_reward{i} = reward;

        % Set speed weight.
        car_reward{i}.theta(3) = car_reward_blank{i}.theta(3);

        % Zero out weight on this car.
        car_reward{i}.theta(ist+i) = 0.0;
    end;

    % Optimize the cars a few times.
    for itrs=1:8,
        % Choose a random shuffle order for this iteration.
        order = randperm(length(cars));
        % Set number of restarts.
        if itrs==1,
            restarts = 4;
        else
            restarts = 2;
        end;
        % Optimize the cars in sequence.
        fprintf(1,'Car optimization iteration %i: ',itrs);
        for i=order,
            % Optimize this car.
            if itrs==1,
                [~,u] = optimizetrajectory(mdp_data.cars{i}.x0,mdp_params.max_time,mdp_data,...
                    [],'highway',car_reward_blank{i},car_reward_blank{i},0,[restarts,400]);
                [x,u] = reoptimizetrajectory(struct('s',mdp_data.cars{i}.x0,'u',u),...
                    mdp_data,'highway',car_reward{i},car_reward{i},[2,200,400]);
                mdp_data.cars{i}.x = x;
                mdp_data.cars{i}.u = u;
            else
                [x,u] = reoptimizetrajectory(struct('s',mdp_data.cars{i}.x0,'u',mdp_data.cars{i}.u),...
                    mdp_data,'highway',car_reward{i},car_reward{i},[restarts,200,400]);
                mdp_data.cars{i}.x = x;
                mdp_data.cars{i}.u = u;
            end;
            fprintf(1,'.');

            % Update all rewards.
            for j=1:length(cars),
                car_reward{j}.features{ist+i}.x = [mdp_data.cars{i}.x0; mdp_data.cars{i}.x];
            end;

            % Update final reward.
            reward.features{ist+i}.x = [mdp_data.cars{i}.x0; mdp_data.cars{i}.x];

            % Update feature.
            features_pt{ist-2+i}.x = [mdp_data.cars{i}.x0; mdp_data.cars{i}.x];
        end;
        fprintf(1,'\n');
    end;
else
    % Load car data from a file.
    load(mdp_params.carfile);
    if length(loaded_cars) ~= length(mdp_data.cars),
        error('Specified car file has the wrong number of cars!');
    end;
    mdp_data.cars = loaded_cars;
    for i=1:length(loaded_cars),
        reward.features{ist+i}.x = [mdp_data.cars{i}.x0; mdp_data.cars{i}.x];
        features_pt{ist-2+i}.x = [mdp_data.cars{i}.x0; mdp_data.cars{i}.x];
    end;
end;

% Move speed into dynamics features.
if mdp_params.linear_speed,
    features_dyn = [features_dyn features_pt(length(features_pt)-length(mdp_data.cars))];
    features_pt(length(features_pt)-length(mdp_data.cars)) = [];
end;

% Build final car features.
features_pt = highwaycarfeatures(features_pt,mdp_data);

% Move center lane into dynamic features.
%features_dyn = [features_dyn features_pt(mdp_params.lanes+1)];
%features_pt(mdp_params.lanes+1) = [];

% Group the lanes into one sum.
if mdp_params.group_lane_features,
    lanes = features_pt(1:mdp_params.lanes);
    features_pt(1:mdp_params.lanes) = [];
    if mdp_params.group_lanes_max,
        lanesum = struct('type','max','min',0,'features',{lanes});
    else
        lanesum = struct('type','sum','theta',ones(1,mdp_params.lanes),'features',{lanes});
    end;
    features_pt = [{lanesum} features_pt];
end;

% Add turning rate.
turn = struct('type','rbf','idx',5,'pos',0,'width',10.0,'r',1.0);
features_pt = [features_pt {turn}];

% Add speeds.
if mdp_params.speed_features,
    speeds = [0.0, 0.05, 0.1];
    for i=1:length(speeds),
        speedfeat = struct('type','rbf','idx',4,'pos',speeds(i),'width',200.0,'r',1.0);
        features_pt = [features_pt {speedfeat}];
    end;
end;
