% Prepare necessary structures to evaluate statistics.
function evalstruct = highwayevalprepare(mdp_params)

% Evaluates the statistics for each example.
% Statistics are:
% mean speed
% number of collisions
% number of steps behind a car
% number of steps in front of a car

% Create car polygons.
mdp_params = highwaydefaultparams(mdp_params);
rw = mdp_params.lane_space*0.5;
car_width = rw*0.5;
car_height = rw*1.0;
CL = 5;
carpoly = [-car_width, -car_height;...
           -car_width, car_height;...
           car_width, car_height;...
           car_width, -car_height;...
           -car_width, -car_height];
carfrontpoly = [-car_width, car_height;...
           -car_width, car_height*CL;...
           car_width, car_height*CL;...
           car_width, car_height;...
           -car_width, car_height];
carbackpoly = [-car_width, -car_height*CL;...
           -car_width, -car_height;...
           car_width, -car_height;...
           car_width, -car_height*CL;...
           -car_width, -car_height*CL];

% Return result.
evalstruct = struct('T',mdp_params.max_time,'car_width',car_width,'car_height',car_height,...
    'carpoly',carpoly,'carfrontpoly',carfrontpoly,'carbackpoly',carbackpoly);
