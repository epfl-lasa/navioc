% Draw single robot arm world with specified reward function.
function robotarmdraw(reward,example_samples,test_samples,mdp_params,mdp_data,paper_quality)

if nargin < 6,
    paper_quality = 0;
end;

% Constants.
ARM_WIDTH = 0.1;
ARM_JOINT_WIDTH = 0.2;
ARM_BASE_WIDTH = 0.3;
EEBL = 0.1;
EEW = 0.25;
EEIW = 0.1;
EETL = 0.4;
if paper_quality ~= 0,
    ARM_SPACING = 4;
    DETAILED_ARM = 1;
    SHOW_END_EFFECTOR = 0;
    SHOW_SINGLE_EXAMPLE = paper_quality;
    SHOW_EXAMPLES = 0;
    SHOW_TESTS = 0;
else
    ARM_SPACING = 16;
    DETAILED_ARM = 0;
    SHOW_END_EFFECTOR = 1;
    SHOW_SINGLE_EXAMPLE = 0;
    SHOW_EXAMPLES = 1;
    SHOW_TESTS = 1;
end;


% Initialize window.
if paper_quality,
    VMARGIN = -1.0;
    HMARGIN = -1.0;
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
% TODO: in the future, we may need to perform inverse kinematics here.
R = feval(strcat(reward.type,'evalreward'),reward,mdp_data,[],zeros(size(pts,1),mdp_data.udims),pts,[],[],[],[]);

% Convert back into image.
C = reshape(R,size(X,1),size(Y,1));

% Interpolate.
C = interp2(C,4);

% Visualize.
C = C - min(min(C));
if max(max(C))~=0
	C = C/max(max(C));
    C = C*64;
	image(x, y, C);
end;

% Draw the paths.
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
    else
        numts = [];
    end;
    for i=numex,
        % Collect all points in this trajectory.
        states = robotarmcontrol(mdp_data,example_samples{i}.s,example_samples{i}.u);
        states = [example_samples{i}.s; states];
        [ptx,pty] = robotarmfk(states,mdp_data);
        col = ones(1,3)*0.0;
        if SHOW_END_EFFECTOR,
            % Plot the end effector trajectory.
            plot(ptx(:,end),pty(:,end),'-','color',col,'marker','.','markersize',6,'linewidth',1);
            % Plot ending point.
            plot(ptx(end,end),pty(end,end),'color',col,'marker','x','markersize',8,'linewidth',2);
        end;
        % Plot the arm at each step.
        col = ones(1,3)*0.5;
        if DETAILED_ARM,
            % Plot a detailed arm.
            pxs = [mdp_data.bounds(1)*0.5 ptx(1,:)];
            pys = [mdp_data.bounds(2)*0.5 pty(1,:)];
            % Draw base.
            basecolor = [0.3 0.3 0.3];
            rectangle('Position',[[pxs(1,1) pys(1,1)]-ARM_BASE_WIDTH 2*ARM_BASE_WIDTH 2*ARM_BASE_WIDTH],...
                    'Curvature',[1 1],'FaceColor',basecolor,'EdgeColor','k');
            for jj=1:2:size(ptx,2),
                for t=1:ARM_SPACING:size(states,1),
                    for j=jj:min(jj+1,size(ptx,2)),
                        pxs = [mdp_data.bounds(1)*0.5 ptx(t,:)];
                        pys = [mdp_data.bounds(2)*0.5 pty(t,:)];
                        % Get origin location.
                        strtpos = [pxs(j) pys(j)];
                        endpos = [pxs(j+1) pys(j+1)];
                        vec = endpos-strtpos;
                        vec = vec./norm(vec);
                        perp = [-vec(2) vec(1)];
                        verts = [strtpos + perp*ARM_WIDTH; endpos + perp*ARM_WIDTH; endpos - perp*ARM_WIDTH; strtpos - perp*ARM_WIDTH];
                        tfrac = (t-1)/(size(states,1)-1);
                        armcolor = [0.7 0.7 0.7]*tfrac+[0.2 0.2 0.2];
                        % Draw link body.
                        patch(verts(:,1),verts(:,2),armcolor,'EdgeColor','k');
                        % Draw circle at starting location.
                        rectangle('Position',[strtpos-ARM_JOINT_WIDTH 2*ARM_JOINT_WIDTH 2*ARM_JOINT_WIDTH],...
                            'Curvature',[1 1],'FaceColor',armcolor,'EdgeColor','k');
                        if j==size(ptx,2),
                            % Draw end effector.
                            verts = [endpos - vec*EEBL + perp*EEW;...
                                     endpos + vec*EEBL + perp*EEW;...
                                     endpos + vec*EETL + perp*EEIW;...
                                     endpos + vec*EEBL + perp*EEIW;...
                                     endpos + vec*EEBL - perp*EEIW;...
                                     endpos + vec*EETL - perp*EEIW;...
                                     endpos + vec*EEBL - perp*EEW;...
                                     endpos - vec*EEBL - perp*EEW;...
                                     endpos - vec*EEBL + perp*EEW];
                            patch(verts(:,1),verts(:,2),armcolor,'EdgeColor','k');
                        end;
                    end;
                end;
            end;
        else
            for t=1:ARM_SPACING:size(states,1),
                plot([mdp_data.bounds(1)*0.5 ptx(t,:)],[mdp_data.bounds(2)*0.5 pty(t,:)],'-','color',col,'marker','.','markersize',6,'linewidth',1.0);
                %plot(ptx(t,end),pty(t,end),'color',col,'marker','x','markersize',8,'linewidth',0.5);
            end;
        end;
    end;
    for i=numts,
        % Collect all points in this trajectory.
        states = robotarmcontrol(mdp_data,test_samples{i}.s,test_samples{i}.u);
        states = [test_samples{i}.s; states];
        [ptx,pty] = robotarmfk(states,mdp_data);
        col = [0.3 0.3 0.7];
        if SHOW_END_EFFECTOR,
            % Plot the end effector trajectory.
            plot(ptx(:,end),pty(:,end),'-','color',col,'marker','.','markersize',6,'linewidth',1);
            % Plot ending point.
            plot(ptx(end,end),pty(end,end),'color',col,'marker','x','markersize',8,'linewidth',2);
        end;
        % Plot the arm at each step.
        %col = ones(1,3)*0.5;
        %for t=1:16:size(states,1),
        %    plot([mdp_data.bounds(1)*0.5 ptx(t,:)],[mdp_data.bounds(2)*0.5 pty(t,:)],'-','color',col,'marker','.','markersize',6,'linewidth',1.0);
        %    %plot(ptx(t,end),pty(t,end),'color',col,'marker','x','markersize',8,'linewidth',0.5);
        %end;
    end;
end;

% Finished.
hold off;
