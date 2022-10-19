% Create highway drawing structure.
function drawstruct = highwaydrawcreate(mdp_data,mdp_params,fill)

if nargin < 3,
    fill = 1;
end;

% Draw background.
grass = [0 0.3 0];
VMARGIN = 1.0;
HMARGIN = 1.0;
xv = [-HMARGIN mdp_data.bounds(1)+HMARGIN mdp_data.bounds(1)+HMARGIN -HMARGIN];
yv = [-VMARGIN -VMARGIN mdp_data.bounds(2)+VMARGIN mdp_data.bounds(2)+VMARGIN];
if fill,
    patch(xv,yv,grass);
end;

% Draw road lanes.
lanecolor = [0.2 0.2 0.2];
rw = mdp_params.lane_space*0.5;
if fill,
    for i=1:length(mdp_data.lanes),
        % Draw all horizontal sections.
        for j=1:size(mdp_data.lanes{i}.xs,1),
            patch([mdp_data.lanes{i}.xs(j,1) mdp_data.lanes{i}.xs(j,1) mdp_data.lanes{i}.xe(j,1) mdp_data.lanes{i}.xe(j,1)],...
                  [mdp_data.lanes{i}.xs(j,2)-rw mdp_data.lanes{i}.xs(j,2)+rw mdp_data.lanes{i}.xe(j,2)+rw mdp_data.lanes{i}.xe(j,2)-rw],...
                  lanecolor,'EdgeColor','none');
        end;
        % Draw all vertical sections.
        for j=1:size(mdp_data.lanes{i}.ys,1),
            patch([mdp_data.lanes{i}.ys(j,1)-rw mdp_data.lanes{i}.ys(j,1)+rw mdp_data.lanes{i}.ye(j,1)+rw mdp_data.lanes{i}.ye(j,1)-rw],...
                  [mdp_data.lanes{i}.ys(j,2) mdp_data.lanes{i}.ys(j,2) mdp_data.lanes{i}.ye(j,2) mdp_data.lanes{i}.ye(j,2)],...
                  lanecolor,'EdgeColor','none');
        end;
        % Draw all circle sections.
        for j=1:size(mdp_data.lanes{i}.cc,1),
            if mdp_data.lanes{i}.cq(j) == 1,
                theta = linspace(-1.0e-2,pi/2+1.0e-2,20);
            elseif mdp_data.lanes{i}.cq(j) == 2,
                theta = linspace(pi/2-1.0e-2,pi+1.0e-2,20);
            elseif mdp_data.lanes{i}.cq(j) == 3,
                theta = linspace(pi-1.0e-2,3*pi/2+1.0e-2,20);
            elseif mdp_data.lanes{i}.cq(j) == 4,
                theta = linspace(3*pi/2-1.0e-2,2*pi+1.0e-2,20);
            end;
            xv = [(mdp_data.lanes{i}.cr(j)-rw)*sin(theta)+mdp_data.lanes{i}.cc(j,1) ...
                  fliplr((mdp_data.lanes{i}.cr(j)+rw)*sin(theta)+mdp_data.lanes{i}.cc(j,1))];
            yv = [(mdp_data.lanes{i}.cr(j)-rw)*cos(theta)+mdp_data.lanes{i}.cc(j,2) ...
                  fliplr((mdp_data.lanes{i}.cr(j)+rw)*cos(theta)+mdp_data.lanes{i}.cc(j,2))];
            patch(xv,yv,lanecolor,'EdgeColor','none');
        end;
    end;
end;
% Draw lane dividers.
if fill,
    edgecolor = [1 1 0];
    lw = 1;
else
    edgecolor = [0 0 0];
    lw = 3;
end;
for i=1:length(mdp_data.lanes),
    % Draw all horizontal sections.
    for j=1:size(mdp_data.lanes{i}.xs,1),
        plot([mdp_data.lanes{i}.xs(j,1) mdp_data.lanes{i}.xe(j,1)],[mdp_data.lanes{i}.xs(j,2)+rw mdp_data.lanes{i}.xe(j,2)+rw],...
             'color',edgecolor,'linewidth',lw);
        plot([mdp_data.lanes{i}.xs(j,1) mdp_data.lanes{i}.xe(j,1)],[mdp_data.lanes{i}.xs(j,2)-rw mdp_data.lanes{i}.xe(j,2)-rw],...
             'color',edgecolor,'linewidth',lw);
    end;
    % Draw all vertical sections.
    for j=1:size(mdp_data.lanes{i}.ys,1),
        plot([mdp_data.lanes{i}.ys(j,1)+rw mdp_data.lanes{i}.ye(j,1)+rw],[mdp_data.lanes{i}.ys(j,2) mdp_data.lanes{i}.ye(j,2)],...
             'color',edgecolor,'linewidth',lw);
        plot([mdp_data.lanes{i}.ys(j,1)-rw mdp_data.lanes{i}.ye(j,1)-rw],[mdp_data.lanes{i}.ys(j,2) mdp_data.lanes{i}.ye(j,2)],...
             'color',edgecolor,'linewidth',lw);
    end;
    % Draw all circle sections.
    for j=1:size(mdp_data.lanes{i}.cc,1),
        if mdp_data.lanes{i}.cq(j) == 1,
            theta = linspace(-1.0e-2,pi/2+1.0e-2,20);
        elseif mdp_data.lanes{i}.cq(j) == 2,
            theta = linspace(pi/2-1.0e-2,pi+1.0e-2,20);
        elseif mdp_data.lanes{i}.cq(j) == 3,
            theta = linspace(pi-1.0e-2,3*pi/2+1.0e-2,20);
        elseif mdp_data.lanes{i}.cq(j) == 4,
            theta = linspace(3*pi/2-1.0e-2,2*pi+1.0e-2,20);
        end;
        xv1 = (mdp_data.lanes{i}.cr(j)-rw)*sin(theta)+mdp_data.lanes{i}.cc(j,1);
        xv2 = (mdp_data.lanes{i}.cr(j)+rw)*sin(theta)+mdp_data.lanes{i}.cc(j,1);
        yv1 = (mdp_data.lanes{i}.cr(j)-rw)*cos(theta)+mdp_data.lanes{i}.cc(j,2);
        yv2 = (mdp_data.lanes{i}.cr(j)+rw)*cos(theta)+mdp_data.lanes{i}.cc(j,2);
        plot(xv1,yv1,'color',edgecolor,'linewidth',lw);
        plot(xv2,yv2,'color',edgecolor,'linewidth',lw);
    end;
end;

% Create all cars.
car_handles = zeros(1,length(mdp_data.cars));
car_width = rw*0.5;
car_height = rw*1.0;
car_color = [1 0.2 0.2];
cara_color = [0.8 0.7 0.4];

% Create struct to return.
drawstruct = struct('car_handles',car_handles,'car_width',car_width,...
    'car_height',car_height,'car_color',car_color,'cara_color',cara_color);
