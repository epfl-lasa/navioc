% Modify the features to add the final set of car features to the highway.
function features_pt = highwaycarfeatures(features_pt,mdp_data)

% Pull out the car features.
cars = features_pt(length(features_pt)-length(mdp_data.cars)+1:end);
features_pt(length(features_pt)-length(mdp_data.cars)+1:end) = [];

% Create standard car feature.
carstd = struct('type','sum','theta',ones(1,length(cars)),'features',{cars});

% Create big forward/backward Gaussian.
for i=1:length(cars),
    cars{i}.width = 50;
end;
carsfb = struct('type','sum','theta',ones(1,length(cars)),'features',{cars});

% Create big sideways Gaussian.
for i=1:length(cars),
    cars{i}.width = 50;
    cars{i}.lam = [1 4];
end;
carslr = struct('type','sum','theta',ones(1,length(cars)),'features',{cars});

% Create forward/backward Gaussian behind car.
offset = [0 -0.1];
for i=1:length(cars),
    cars{i}.width = 50;
    cars{i}.lam = [8 1];
    for t=1:size(cars{i}.x,1),
        theta = cars{i}.x(t,3);
        x = cars{i}.x(t,1) + offset(1)*cos(theta) + offset(2)*sin(theta);
        y = cars{i}.x(t,2) + offset(2)*cos(theta) - offset(1)*sin(theta);
        cars{i}.x(t,1) = x;
        cars{i}.x(t,2) = y;
    end;
end;
carsb = struct('type','sum','theta',ones(1,length(cars)),'features',{cars});

% Create forward/backward Gaussian in front of car.
offset = [0 0.2];
for i=1:length(cars),
    cars{i}.width = 50;
    cars{i}.lam = [8 1];
    for t=1:size(cars{i}.x,1),
        theta = cars{i}.x(t,3);
        x = cars{i}.x(t,1) + offset(1)*cos(theta) + offset(2)*sin(theta);
        y = cars{i}.x(t,2) + offset(2)*cos(theta) - offset(1)*sin(theta);
        cars{i}.x(t,1) = x;
        cars{i}.x(t,2) = y;
    end;
end;
carsf = struct('type','sum','theta',ones(1,length(cars)),'features',{cars});

% Create features.
features_pt = [features_pt {carsf, carsb, carslr, carsfb, carstd}];
