% Draw single objectworld with specified reward function.
function objectworlddraw(reward,example_samples,test_samples,mdp_params,mdp_data,paper_quality)

if nargin < 6,
    paper_quality = 0;
end;

% Initialize window.
if paper_quality,
    VMARGIN = 0.4;
    HMARGIN = 0.4;
else
    VMARGIN = 1.0;
    HMARGIN = 1.0;
end;
axis([-HMARGIN  mdp_data.bounds(1)+HMARGIN  -VMARGIN  mdp_data.bounds(2)+VMARGIN]);
if paper_quality,
    % Expand axes.
    set(gca,'position',[0 0 1 1]);
    set(gca,'color','none','xgrid','off','ygrid','off','visible','off');
    set(gca,'xtick',[]);
    set(gca,'ytick',[]);
else
    set(gca,'xtick',0:mdp_data.bounds(1));
    set(gca,'ytick',0:mdp_data.bounds(2));
end;
daspect([1 1 1]);
hold on;

% Draw the reward function.
% Create regular samples.
STEP_SIZE = 0.2;
x = -HMARGIN:STEP_SIZE:(mdp_data.bounds(1)+HMARGIN);
y = -VMARGIN:STEP_SIZE:(mdp_data.bounds(2)+VMARGIN);
[X,Y] = meshgrid(x,y);
pts = [X(:) Y(:)];
R = feval(strcat(reward.type,'evalreward'),reward,mdp_data,[],zeros(size(pts,1),mdp_data.udims),pts * mdp_data.sensor_basis,[],[],[],[]);

% Convert back into image.
C = reshape(R,size(X,1),size(X,2));

% Interpolate.
C = interp2(C,4);

% Visualize.
C = C - min(min(C));
if max(max(C))~=0
	C = C/max(max(C));
    C = C*64;
	image(x, y, C);
end;

% Draw feature positions.
objectworlddrawfeature(reward,0.0,0.0,1.0,[],[]);

if strcmp(reward.type,'sum'),
    max_r = -Inf;
    min_r = Inf;
    for i=1:length(reward.features),
        if strcmp(reward.features{i}.type,'rbf'),
            if reward.theta(i) > max_r,
                max_r = reward.theta(i);
            end;
            if reward.theta(i) < min_r;
                min_r = reward.theta(i);
            end;
        end;
    end;
    rng_r = max_r-min_r;
    for i=1:length(reward.features),
        if strcmp(reward.features{i}.type,'rbf'),
            c = (reward.theta(i)-min_r)/rng_r;
            w = sqrt(1.0/reward.features{i}.width);
            strt = [reward.features{i}.pos(:,1) reward.features{i}.pos(:,end)] - w;
            extn = w*2.0;
            rectangle('Position',[strt(1) strt(2) extn extn],'Curvature',[1 1],'EdgeColor',[c c c]);
        end;
    end;
end;

% Draw the paths.
if ~isempty(example_samples),
    for i=1:length(example_samples),
        % Collect all points in this trajectory.
        pts = objectworldcontrol(mdp_data,example_samples{i}.s,example_samples{i}.u);
        pts = [example_samples{i}.s; pts];
        T = size(pts,1);
        col = ones(1,3)*0.0;
        % Plot the points.
        if paper_quality,
            width_factor = 2;
        else
            width_factor = 1;
        end;
        plot(pts(:,1),pts(:,end),'-','color',col,'marker','.','markersize',14*width_factor,'linewidth',1.5);
        % Plot starting point.
        %plot(pts(1,1),pts(1,end),'color',col,'marker','o','markersize',5,'linewidth',2);
        % Plot ending point.
        plot(pts(end,1),pts(end,end),'color',col,'marker','x','markersize',10*width_factor,'linewidth',2);
    end;
    if ~paper_quality,
    for i=1:length(test_samples),
        % Collect all points in this trajectory.
        pts = objectworldcontrol(mdp_data,test_samples{i}.s,test_samples{i}.u);
        pts = [test_samples{i}.s; pts];
        col = [0.5 0.5 0.7];
        % Plot the points.
        plot(pts(:,1),pts(:,end),'-','color',col,'marker','.','markersize',14,'linewidth',1.5);
        % Plot starting point.
        %plot(pts(1,1),pts(1,end),'color',col,'marker','o','markersize',5,'linewidth',2);
        % Plot ending point.
        plot(pts(end,1),pts(end,end),'color',col,'marker','x','markersize',10,'linewidth',2);
    end;
    end;
end;

% Finished.
hold off;
