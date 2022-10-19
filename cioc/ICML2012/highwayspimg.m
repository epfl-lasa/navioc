% Make highway images.
function highwayspimg(file)

% Load the file
load(file);

T = 16;
    
% Compute view area.
FOLLOWHSIZE = 0.4;
FOLLOWVSIZE = 0.32;
pos = [9.0 5.8];
rngx = [pos(1)-FOLLOWHSIZE pos(1)+FOLLOWHSIZE];
rngy = [pos(2)-FOLLOWVSIZE pos(2)+FOLLOWVSIZE];

STEPSX = 50;%100
STEPSY = 40;%80
x = linspace(rngx(1),rngx(2),STEPSX);
y = linspace(rngy(1),rngy(2),STEPSY);
[X,Y] = meshgrid(x,y);
pts = [X(:) Y(:) zeros(numel(X),4)];
pts(:,6) = T;
    
% Prepare each reward.
i = 1;
steps = -0.10:0.025:0.20;
R = zeros(size(pts,1),size(steps,2));
for s=steps,
    pts(:,4) = s;
    R(:,i) = feval(strcat(test_result.irl_result.reward.type,'evalreward'),...
        test_result.irl_result.reward,test_result.mdp_data,[],zeros(size(pts,1),test_result.mdp_data.udims),pts,[],[],[],[]);
    i = i + 1;
end;

% Normalize.
minr = min(min(R));
maxr = max(max(R))-minr;

% Render each image.
i = 1;
for s=steps,
    % Create the window.
    w = 400;
    h = 320;
    figure('Position',[700 50 w h]);
    hold on;
    gca;
    set(gca,'position',[0 0 1 1]);
    set(gca,'color','none','xgrid','off','ygrid','off','visible','off');
    set(gca,'xtick',[]);
    set(gca,'ytick',[]);
    
    % Draw reward.
    C = reshape(R(:,i),size(y,2),size(x,2));
    i = i + 1;
    C = interp2(C,2);
    C = C - minr;
    C = C/maxr;
    C = C*64;
    image(x, y, C);
    fill = 0;
    
    % Draw frame.
    mdp_data = test_result.mdp_data;
    mdp_params = test_result.mdp_params;
    mdp_params = highwaydefaultparams(mdp_params);
    drawstruct = highwaydrawcreate(mdp_data,mdp_params,fill);
    drawstruct = highwaydrawmodify(drawstruct,mdp_data,mdp_params,0,fill);
    highwaydrawmodify(drawstruct,mdp_data,mdp_params,T);
    
    % Set view area.
    set(gca,'xlim',rngx);
    set(gca,'ylim',rngy);
end;
