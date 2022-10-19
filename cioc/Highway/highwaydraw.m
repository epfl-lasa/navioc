% Draw highway environment with specified reward function.
function highwaydraw(reward,example_samples,test_samples,mdp_params,mdp_data,paper_quality)

if nargin < 6,
    paper_quality = 0;
end;

% Fill in default parameters.
mdp_params = highwaydefaultparams(mdp_params);

SHOW_REWARD = 0;
FOLLOW = 0;
FOLLOWEX = 0;
FOLLOWHSIZE = 1;
FOLLOWVSIZE = 1;

% Constants.
if paper_quality ~= 0,
    SHOW_SINGLE_EXAMPLE = paper_quality;
    SHOW_EXAMPLES = 0;
    SHOW_TESTS = 0;
else
    SHOW_SINGLE_EXAMPLE = 0;
    SHOW_SINGLE_TEST = 0;
    SHOW_EXAMPLES = 1;
    SHOW_TESTS = 1;
end;

% Initialize window.
VMARGIN = 1.0;
HMARGIN = 1.0;
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

% Create regular samples.
STEP_SIZE = 0.025;%0.025;
x = -HMARGIN:STEP_SIZE:(mdp_data.bounds(1)+HMARGIN);
y = -VMARGIN:STEP_SIZE:(mdp_data.bounds(2)+VMARGIN);
[X,Y] = meshgrid(x,y);
pts = [X(:) Y(:) zeros(numel(X),4)];

% Collect paths.
if ~isempty(example_samples),
    if SHOW_EXAMPLES,
        numex = 1:length(example_samples);
    elseif any(SHOW_SINGLE_EXAMPLE),
        numex = SHOW_SINGLE_EXAMPLE;
    else
        numex = [];
    end;
    if SHOW_TESTS,
        numts = 1:length(test_samples);
    elseif any(SHOW_SINGLE_TEST),
        numts = SHOW_SINGLE_TEST;
    else
        numts = [];
    end;
    ex = cell(1,length(numex));
    tx = cell(1,length(numts));
    j = 1;
    for i=numex,
        % Collect all points in this trajectory.
        states = highwaycontrol(mdp_data,example_samples{i}.s,example_samples{i}.u);
        ex{j} = [example_samples{i}.s; states];
        j = j+1;
    end;
    j = 1;
    for i=numts,
        % Collect all points in this trajectory.
        states = highwaycontrol(mdp_data,test_samples{i}.s,test_samples{i}.u);
        tx{j} = [test_samples{i}.s; states];
        j = j+1;
    end;
    explot = zeros(1,length(ex));
    tsplot = zeros(1,length(tx));
end;

% Create rendering.
drawstruct = highwaydrawcreate(mdp_data,mdp_params);
car_width = drawstruct.car_width;
car_height = drawstruct.car_height;

if ~SHOW_REWARD,
% Now render the animation.
for t=0:mdp_params.max_time,
    % Check if we are following a particular example.
    if FOLLOW,
        if FOLLOWEX,
            pos = ex{FOLLOW}(t+1,1:2);
        else
            pos = tx{FOLLOW}(t+1,1:2);
        end;
        set(gca,'xlim',[pos(1)-FOLLOWHSIZE pos(1)+FOLLOWHSIZE]);
        set(gca,'ylim',[pos(2)-FOLLOWVSIZE pos(2)+FOLLOWVSIZE]);
    end;
    
    % Draw the static image.
    if SHOW_REWARD,
        pts(:,6) = t;
        R = feval(strcat(reward.type,'evalreward'),reward,mdp_data,[],zeros(size(pts,1),mdp_data.udims),pts,[],[],[],[]);
        C = reshape(R,size(X,1),size(Y,1));
        C = interp2(C,2);
        C = C - min(min(C));
        if max(max(C))~=0
            C = C/max(max(C));
            C = C*64;
            image(x, y, C);
        end;
    end;
    
    % Render all cars.
    drawstruct = highwaydrawmodify(drawstruct,mdp_data,mdp_params,t);
    
    % Now render the examples.
    if ~isempty(example_samples),
        for j=1:length(ex),
            idx = t+1;
            if size(ex{j},1) >= idx,
                pos = ex{j}(idx,1:2);
                theta = ex{j}(idx,3);
                col = [0.0 0.3 1.0];
                [xv,yv] = highwaycarplot(pos,theta,drawstruct.car_width,drawstruct.car_height);
                if t > 0,
                    set(explot(j),'xdata',xv);
                    set(explot(j),'ydata',yv);
                else
                    explot(j) = patch(xv,yv,col,'edgecolor','k');
                end;
            end;
        end;
        for j=1:length(tx),
            idx = t+1;
            if size(tx{j},1) >= idx,
                pos = tx{j}(idx,1:2);
                theta = tx{j}(idx,3);
                col = [0.5 0.5 0.8];
                [xv,yv] = highwaycarplot(pos,theta,drawstruct.car_width,drawstruct.car_height);
                if t > 0,
                    set(tsplot(j),'xdata',xv);
                    set(tsplot(j),'ydata',yv);
                else
                    tsplot(j) = patch(xv,yv,col,'edgecolor','k');
                end;
            end;
        end;
    end;
    pause(0.1);
end;
end;

% Now draw the static image.
if SHOW_REWARD,
    pts(:,6) = 16;
    pts(:,4) = 0.1;
    R = feval(strcat(reward.type,'evalreward'),reward,mdp_data,[],zeros(size(pts,1),mdp_data.udims),pts,[],[],[],[]);
    C = reshape(R,size(X,1),size(Y,1));
    C = interp2(C,2);
    C = C - min(min(C));
    if max(max(C))~=0
        C = C/max(max(C));
        C = C*64;
        image(x, y, C);
    end;
end;

% Draw the paths.
if ~isempty(example_samples),
    for j=1:length(ex),
        % Collect all points in this trajectory.
        states = ex{j};
        col = [0.0 0.3 1.0];
        % Plot the trajectory.
        plot(states(:,1),states(:,2),'-','color',col,'marker','.','markersize',6,'linewidth',1);
        % Plot ending point.
        plot(states(end,1),states(end,2),'color',col,'marker','x','markersize',8,'linewidth',2);
    end;
    for j=1:length(tx),
        % Collect all points in this trajectory.
        states = tx{j};
        col = [0.5 0.5 0.8];
        % Plot the trajectory.
        plot(states(:,1),states(:,2),'-','color',col,'marker','.','markersize',6,'linewidth',1);
        % Plot ending point.
        plot(states(end,1),states(end,2),'color',col,'marker','x','markersize',8,'linewidth',2);
    end;
end;

% Finished.
hold off;
