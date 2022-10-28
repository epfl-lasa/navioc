function tracks = importdata(dataset_name, tracks_name, info_name)
% IMPORTDATA  Import data from a dataset in ../Datasets.
%   tracks = IMPORTDATA(dataset_name, tracks_name, info_name) loads
%   data from the dataset with the given name dataset_name, name of
%   the tracks-file tracks_name, and name of the info-file info_name,
%   and returns the cell-array tracks, which contains one entry per
%   agent, storing its type and cartesian (and possibly angular)
%   positions over time.

dataset_name = string(dataset_name);
tracks_name = string(tracks_name);

tracks_path = "../Datasets/" + dataset_name + "/tracks/" + tracks_name;

if nargin > 2
    info_name = string(info_name);
    info_path = "../Datasets/" + dataset_name + "/info/" + info_name;
end

switch dataset_name
case "diamor-atc"
    tracks = diamor_atc(tracks_path, info_path);
case "eth"
    tracks = eth(tracks_path);
case "pamela"
    tracks = pamela(tracks_path, char(tracks_name));
case "stanford-drone-death-circle"
    tracks = stanford_drone_death_circle(tracks_path, char(tracks_name));
case "ucy"
    tracks = ucy(tracks_path, 36.0); % pix/m ratio is a crude estimate
case "vci-citr"
    tracks = vci(tracks_path, 2997/100);
case "vci-dut"
    tracks = vci(tracks_path, 2997/125);
otherwise
    fprintf ("Unrecognized dataset %s.\n", dataset_name);
    tracks = {};
end


function tracks = eth(tracks_path)

frame_rate = 2.5*6.0;

data = readtable(tracks_path);

ids = unique(data{:, 2});

n = length(ids);

tracks = cell(1, n);
for i = 1:n
    tracks{i} = struct('type', "ped");

    agent_data = data(ids(i) == data{:, 2}, [1, 3, 5, 6, 8]);
    tracks{i}.t = agent_data{:, 1}/frame_rate;
    tracks{i}.x = agent_data{:, 2};
    tracks{i}.y = agent_data{:, 3};
    tracks{i}.vx = agent_data{:, 4};
    tracks{i}.vy = agent_data{:, 5};
end


function tracks = stanford_drone_death_circle(tracks_path, tracks_name)

frame_rate = 2997/100;

switch tracks_name(1)
case '0'
    pixels_per_meter = 171/5.1;
case '1'
    pixels_per_meter = 175/5.1;
case '2'
    pixels_per_meter = 175/5.1;
case '3'
    pixels_per_meter = 247/5.1;
case '4'
    pixels_per_meter = 182/5.1;
end

data = readtable(tracks_path);

ids = unique(data{:, 1});

n = length(ids);

tracks = cell(1, n);

for i = 1:n
    tracks{i} = struct();

    agent_data = data(ids(i) == data{:, 1}, 2:end);

    tracks{i}.t = agent_data{:, 5}/frame_rate;
    tracks{i}.x = (agent_data{:, 1} + agent_data{:, 3})/2/pixels_per_meter;
    tracks{i}.y = (agent_data{:, 2} + agent_data{:, 4})/2/pixels_per_meter;

    switch string(agent_data{1, 9})
    case "Pedestrian"
        tracks{i}.type = "ped";
    case "Biker"
        tracks{i}.type = "bicycle";
    case "Skater"
        tracks{i}.type = "skateboard";
    case "Cart"
        tracks{i}.type = "car";
    case "Car"
        tracks{i}.type = "car";
    case "Bus"
        tracks{i}.type = "bus";
    otherwise
        error("Unrecognized agent label %s.", agent_data{1, 9}{1})
    end
end


function tracks = pamela(tracks_path, tracks_name)

frame_rate = 12.5;

switch tracks_name(2)
case {'2', '3'}
    robot_type = "powered_wheelchair";
case '4'
    robot_type = "pepper_robot";
end

fid = fopen(tracks_path);

fgetl(fid);

words = strsplit(fgetl(fid));

robot_id = str2double(words{5});

fclose(fid);

data = readtable(tracks_path, 'HeaderLines', 3);

ids = unique(data{:, 1});

n = length(ids);

tracks = cell(1, n);

for i = 1:n
    tracks{i} = struct();

    if ids(i) == robot_id
        tracks{i}.type = robot_type;
    else
        tracks{i}.type = "ped";
    end

    agent_data = data(ids(i) == data{:, 1}, 2:4);

    tracks{i}.t = agent_data{:, 1}/frame_rate;
    tracks{i}.x = agent_data{:, 2};
    tracks{i}.y = agent_data{:, 3};
end


function tracks = ucy(tracks_path, pixels_per_meter)

frame_rate = 25;

fid = fopen(tracks_path);

words = strsplit(fgetl(fid));

n = str2double(words{1}); % number of agents

tracks = cell(1, n);

for i = 1:n
    words = strsplit(fgetl(fid));

    m = str2double(words{1}); % number of points

    tracks{i} = struct('type', "ped", 't', zeros(m, 1), ...
        'x', zeros(m, 1), 'y', zeros(m, 1));

    for j = 1:m
        words = strsplit(fgetl(fid));

        tracks{i}.t(j) = str2double(words{3})/frame_rate;
        tracks{i}.x(j) = str2double(words{1})/pixels_per_meter;
        tracks{i}.y(j) = str2double(words{2})/pixels_per_meter;
    end
end

fclose(fid);


function tracks = vci(tracks_path, frame_rate)

data = readtable(tracks_path);

ids = unique(data{:, 1});

n = length(ids); % number of agents

tracks = cell(1, n);

for i = 1:n
    tracks{i} = struct();

    agent_data = data(ids(i) == data{:, 1}, 2:end);
switch string(agent_data{1, 2})
    case "ped"
        tracks{i}.type = "ped";
    case "veh"
        tracks{i}.type = "car";
        tracks{i}.phi = agent_data{:, 5};
    otherwise
        error("Unrecognized agent label %s.", agent_data{1, 2})
end

    tracks{i}.t = agent_data{:, 1}/frame_rate;
    tracks{i}.x = agent_data{:, 3};
    tracks{i}.y = -agent_data{:, 4};
end


function tracks = diamor_atc(tracks_path, info_path)

fid = fopen(info_path);

hand_driven_wheelchair_ids = [];
automatic_wheelchair_ids = [];
baby_carriage_ids = [];
wheelchair_companion_ids = [];

while true
    thisline = fgetl(fid);
    
    if ~ischar(thisline)
        break % reached end of file
    end

    words = strsplit(thisline);
    agent_id = str2double(words{1});

    if agent_id < -1
        agent_id = -agent_id;
        second_number = str2double(words{2});
        if second_number > 0
            group_size = second_number;
            baby_carriage_ids = [baby_carriage_ids, agent_id];
        else
            group_size = str2double(words{3});
            if second_number == -2
                hand_driven_wheelchair_ids = [hand_driven_wheelchair_ids, agent_id];
            elseif second_number == -3
                automatic_wheelchair_ids = [automatic_wheelchair_ids, agent_id];
            end
            for i = 1:(group_size - 1)
                wheelchair_companion_ids = [wheelchair_companion_ids, ...
                    str2double(words{3 + i})];
            end
        end
        % if second_number == -2
        %     hand_driven_wheelchair_ids = [hand_driven_wheelchair_ids, agent_id];
        % elseif second_number == -3
        %     automatic_wheelchair_ids = [automatic_wheelchair_ids, agent_id];
        % elseif second_number > 0
        %     baby_carriage_ids = [baby_carriage_ids, agent_id];
        % end
    end
end

fclose(fid);

data = readtable(tracks_path);

ids = unique(data{:, 2});

n = length(ids); % number of agents

tracks = cell(1, n);

for i = 1:n
    tracks{i} = struct();

    agent_data = data(ids(i) == data{:, 2}, [1, 3, 4, 8]);

    if ismember(ids(i), hand_driven_wheelchair_ids)
        tracks{i}.type = "manual_wheelchair";
        tracks{i}.phi = agent_data{:, 4};
    elseif ismember(ids(i), automatic_wheelchair_ids)
        tracks{i}.type = "powered_wheelchair";
        tracks{i}.phi = agent_data{:, 4}; 
    elseif ismember(ids(i), baby_carriage_ids)
        tracks{i}.type = "stroller";
        tracks{i}.phi = agent_data{:, 4};
    elseif ismember(ids(i), wheelchair_companion_ids)
        tracks{i}.type = "wheelchair_companion";
    else
        tracks{i}.type = "ped";
    end

    tracks{i}.t = agent_data{:, 1};
    tracks{i}.x = agent_data{:, 2}/1000;
    tracks{i}.y = agent_data{:, 3}/1000;
end