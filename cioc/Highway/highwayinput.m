% Open highway user input window and record user input.
function example = highwayinput(mdp_data,mdp_params,seed,input_type,oldex)

% Set seed.
rng(abs(seed));

% Fill in default parameters.
mdp_params = highwaydefaultparams(mdp_params);

% Create window.
w = 900;
h = 900;
figure('Position',[700 50 w h]);
hold on;
gca;
set(gca,'position',[0 0 1.0 1.0]);

% Draw first frame.
drawstruct = highwaydrawcreate(mdp_data,mdp_params);
drawstruct = highwaydrawmodify(drawstruct,mdp_data,mdp_params,0);

% Sample initial state.
if nargin >= 5,
    x = oldex.s;
else
    x = highwaysamplestates(1,mdp_data);
end;
if seed < 0,
    % If the seed is negative, place the initial state off the grass.
    x(1:2) = x(1:2) + randn(1,2)*0.2;
end;
x0 = x;
col = [0.0 0.3 1.0];
[xv,yv] = highwaycarplot(x(1:2),x(3),drawstruct.car_width,drawstruct.car_height);
carplot = patch(xv,yv,col,'edgecolor','k');

% Create speed text.
speed = text(x(1)+0.2*cos(x(3)),x(2)-0.2*sin(x(3)),num2str(x(4)),'FontSize',16,'color',[1 0 0]);

% Set viewpoint.
FOLLOWHSIZE = 1;
FOLLOWVSIZE = 1;
VMARGIN = 1.0;
HMARGIN = 1.0;
set(gca,'xlim',[-HMARGIN  mdp_data.bounds(1)+HMARGIN]);
set(gca,'ylim',[-VMARGIN  mdp_data.bounds(2)+VMARGIN]);
camroll(rad2deg(x(3)));
set(gca,'CameraPosition',[x(1) x(2) 0.2]);
set(gca,'CameraTarget',[x(1) x(2) 0]);
set(gca,'CameraViewAngle',90);
olddeg = x(3);

% Wait for mouse click.
waitforbuttonpress;

% Try to get 3D mouse.
if input_type == 1,
    try
        m = Mouse3D('init');
        if isreal(m),
            m = Mouse3D('init');
        end;
    catch
        fprintf(1,'No 3D mouse detected! Using normal mouse.\n');
        m = [];
    end;
else
    m = [];
end;

ax0 = gca;
if isempty(m),
    % Get initial mouse position.
    pt = get(0,'PointerLocation');
    SCALE = 5.0e-2;
    % Create steering wheel icon.
    WS = 50;
    p = get(gcf,'position');
    ax1 = axes('position',[(pt(1)-p(1)-WS)/p(3) (pt(2)-p(2)-WS)/p(4) WS*2/p(3) WS*2/p(4)],'units','pixels');
    axis([-1 1 -1 1]);
    % Create indicators for old and current input.
    hold on;
    set(ax1,'color','none','xgrid','off','ygrid','off','visible','on');
    set(ax1,'xtick',[]);
    set(ax1,'ytick',[]);
    ptsq = [-1, -1; -1 1; 1 1; 1 -1];
    patch(ptsq(:,1),ptsq(:,2),'w','edgecolor','k');
    line([-1 1],[0 0],'color','k');
    line([0 0],[-1 1],'color','k');
    oldinput = plot(0,0,'b.','MarkerSize',10);
    curinput = plot(0,0,'r.','MarkerSize',10);
end;

% Start stepping.
if nargin >= 5,
    u = oldex.u;
else
    u = zeros(mdp_params.max_time,2);
end;
states = zeros(mdp_params.max_time,6);
t = 0;
while t < mdp_params.max_time,
    if input_type == 0,
        % Wait for button press.
        k = waitforbuttonpress;
        c = get(gcf,'CurrentCharacter');
        
        % Decide which action to take.
        if k == 0, % Mouse press - advance and use the input.
            t = t + 1;
            update_input = 1;
        elseif c == 'w',
            t = t + 1;
            update_input = 0;
        elseif c == 's',
            if t >= 2,
                t = t - 1;
            end;
            update_input = 0;
            if t > 1,
                x = states(t-1,:);
            else
                x = x0;
            end;
        else
            t = t + 1;
            update_input = 1;
        end;
        %pause(0.2);
    else
        % Wait for time to advance.
        t = t + 1;
        update_input = 1;
        pause(0.2);
    end;
    
    if update_input,
        % Check for input.
        if ~isempty(m),
            data = Mouse3D('get');
            GAS_SCALE = 6.0e-4;
            WHEEL_SCALE = 4.0e-2;
            wheel = -data.rot(2)*data.ang*WHEEL_SCALE;
            ut = [-data.pos(2)*data.len*GAS_SCALE...
                  (wheel-x(5)*mdp_data.cardamp(2))*mdp_data.carmass(2)];
            %fprintf(1,'angle: %f gas: %f\n',ut(2),ut(1));
        else
            npt = get(0,'PointerLocation');
            diff = npt-pt;
            %pt = npt;
            ut = [diff(2)*SCALE,...
                  (diff(1)*SCALE-x(5)*mdp_data.cardamp(2))*mdp_data.carmass(2)];
        end;
    else
        ut = u(t,:);
    end;

    % Compute new state.
    nx = highwaycontrol(mdp_data,x,ut);
    dx = nx(1:3)-x(1:3);
    x = nx;
    states(t,:) = x;
    u(t,:) = ut;
    
    % Update viewpoint.
    dchng = olddeg-x(3);
    olddeg = x(3);
    camroll(ax0,rad2deg(-dchng));
    set(ax0,'CameraPosition',[x(1) x(2) 0.2]);
    set(ax0,'CameraTarget',[x(1) x(2) 0]);
    set(ax0,'CameraViewAngle',90);
    
    % Update position of steering wheel
    if isempty(m),
        if t > 0,
            if t == 1,
                sx = x0;
            else
                sx = states(t-1,:);
            end;
            set(oldinput,'xdata',((u(t,2)/mdp_data.carmass(2) + sx(5)*mdp_data.cardamp(2))/SCALE)/WS);
            set(oldinput,'ydata',u(t,1)/SCALE/WS);
        else
            set(oldinput,'xdata',0);
            set(oldinput,'ydata',0);
        end;
        if t < mdp_params.max_time,
            set(curinput,'xdata',((u(t+1,2)/mdp_data.carmass(2) + states(t,5)*mdp_data.cardamp(2))/SCALE)/WS);
            set(curinput,'ydata',u(t+1,1)/SCALE/WS);
        else
            set(curinput,'xdata',0);
            set(curinput,'ydata',0);
        end;
    end;
    
    % Update drawing.
    [xv,yv] = highwaycarplot(x(1:2),x(3),drawstruct.car_width,drawstruct.car_height);
    drawstruct = highwaydrawmodify(drawstruct,mdp_data,mdp_params,t);
    set(carplot,'xdata',xv);
    set(carplot,'ydata',yv);
    
    % Modify text printout.
    set(speed,'string',num2str(x(4)));
    set(speed,'position',[x(1)+0.2*cos(x(3)) x(2)-0.2*sin(x(3))]);
    
    % Print progress.
    fprintf(1,'Step %i of %i\n',t,mdp_params.max_time);
end;

if ~isempty(m),
    Mouse3D('stop',m);
end;

% Create and return example.
example = struct('s',x0,'u',u,'r',0);
