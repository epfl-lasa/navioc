% Construct the Objectworld MDP structures.
function [mdp_data,reward,features_pt,features_dyn] = objectworldbuild(mdp_params)

% mdp_params - parameters of the objectworld
% mdp_data - standard MDP definition structure with object-world details
% reward - reward object for this objectworld
% features - cell array of features

% Fill in default parameters.
mdp_params = objectworlddefaultparams(mdp_params);

% Set random seed.
rng(mdp_params.seed);

% Compute motor and sensor bases.
motor_basis = zeros(mdp_params.motors,2);
sensor_basis = zeros(2,mdp_params.sensors);
for i=1:mdp_params.motors,
    % Choose angle.
    angle = (pi * 0.5) * (i-1) / (mdp_params.motors - 1);
    % Write basis vector.
    motor_basis(i,:) = [cos(angle) sin(angle)];
end;
for i=1:mdp_params.sensors,
    % Choose angle.
    angle = (pi * 0.5) * (i-1) / (mdp_params.sensors - 1);
    % Write basis vector.
    sensor_basis(:,i) = [cos(angle); sin(angle)];
end;

% Compute pseudoinverse of sensor basis.
sensor_pseudoinverse = sensor_basis'*inv(sensor_basis*sensor_basis');

% Compute state and action bounds.
sp = linspace(0,pi*0.5,mdp_params.sensors);
sbounds = [zeros(1,mdp_params.sensors); (cos(sp) + sin(sp))*mdp_params.size];
abounds = [-ones(1,mdp_params.motors)*0.6; ones(1,mdp_params.motors)*0.6];

% Place the objects.
objects = struct('pos',[],'c1',[],'c2',[]);
if mdp_params.fixed_pattern == 1,
    % Place objects in flower petals fixed pattern.
    objects(1) = struct('pos',[0.5 0.5]*mdp_params.size,'c1',1,'c2',1);
    objects(2) = struct('pos',[0.3 0.3]*mdp_params.size,'c1',2,'c2',1);
    objects(3) = struct('pos',[0.7 0.3]*mdp_params.size,'c1',2,'c2',1);
    objects(4) = struct('pos',[0.3 0.7]*mdp_params.size,'c1',2,'c2',1);
    objects(5) = struct('pos',[0.7 0.7]*mdp_params.size,'c1',2,'c2',1);
elseif mdp_params.fixed_pattern == 2,
    % Place objects in hill and valley fixed pattern.
    i = 1;
    objects(i) = struct('pos',[0.0 0.65]*mdp_params.size,'c1',1,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.1 0.65]*mdp_params.size,'c1',1,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.2 0.65]*mdp_params.size,'c1',1,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.3 0.65]*mdp_params.size,'c1',1,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.4 0.65]*mdp_params.size,'c1',1,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.5 0.65]*mdp_params.size,'c1',1,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.6 0.65]*mdp_params.size,'c1',1,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.7 0.65]*mdp_params.size,'c1',1,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.8 0.65]*mdp_params.size,'c1',1,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.9 0.65]*mdp_params.size,'c1',1,'c2',1);i=i+1;
    objects(i) = struct('pos',[1.0 0.65]*mdp_params.size,'c1',3,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.0 0.35]*mdp_params.size,'c1',4,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.1 0.35]*mdp_params.size,'c1',4,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.2 0.35]*mdp_params.size,'c1',4,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.3 0.35]*mdp_params.size,'c1',4,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.4 0.35]*mdp_params.size,'c1',4,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.5 0.35]*mdp_params.size,'c1',4,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.6 0.35]*mdp_params.size,'c1',4,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.7 0.35]*mdp_params.size,'c1',4,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.8 0.35]*mdp_params.size,'c1',4,'c2',1);i=i+1;
    objects(i) = struct('pos',[0.9 0.35]*mdp_params.size,'c1',4,'c2',1);i=i+1;
    objects(i) = struct('pos',[1.0 0.35]*mdp_params.size,'c1',5,'c2',1);i=i+1;
else
    % Place objects at random.
    for i=1:mdp_params.objects,
        pos = rand(1,2)*mdp_params.size;
        % Select c1 based on probabilities in the parameters.
        samp = rand(1,1);
        if samp < mdp_params.c1_1prob,
            c1 = 1;
        elseif samp < mdp_params.c1_1prob+mdp_params.c1_2prob,
            c1 = 2;
        else
            c1 = 2+randi(mdp_params.c1-2,1);
        end;
        c2 = randi(mdp_params.c2,1);
        objects(i) = struct('pos',pos * sensor_basis,'c1',c1,'c2',c2);
    end;
end;

% Create MDP data structure.
mdp_data = struct(...
    'mdp','objectworld',...
    'dims',size(sensor_basis,2),...
    'udims',size(motor_basis,1),...
    'sbounds',sbounds,'abounds',abounds,...
    'sensor_pseudoinverse',sensor_pseudoinverse,...
    'sensor_basis',sensor_basis,...
    'motor_basis',motor_basis,...
    'bounds',mdp_params.size*ones(1,2),...
    'objects',objects);

% Construct feature map.
[reward,features_pt,features_dyn] = objectworldfeatures(mdp_params,mdp_data);
