% Perform all highway evaluations.
function highwayeval

% These are the files that contain the examples we should render into the
% final video.
files = {'Results/caragg128.mat',...
         'Results/carsaf128.mat',...
         'Results/cartgs128.mat'};
names = {'Aggressive Driver',...
         'Evasive Driver',...
         'Tailgating Driver'};
example_idx = {{6 ,1,1,2,2},...
               {12,2,2,4,4},...
               {8 ,4,4,3,3}};
example_type = {{1,2,3,2,3},...
                {1,2,3,2,3},...
                {1,2,3,2,3}};
cnt = {{1,1,1,2,2},...
       {1,1,1,2,2},...
       {1,1,1,2,2}};
outpath = 'Results/Video/';
SAVE = 1;
CUTOFFTAIL = 16;

% Constants.
TITLE_FRAMES = 20;

% Create window.
w = 800;
h = 800;
FOLLOWHSIZE = 1;
FOLLOWVSIZE = 1;
figure('Position',[700 50 w h]);
hold on;
gca;
set(gca,'position',[0 0 1 1]);
set(gca,'color','none','xgrid','off','ygrid','off','visible','off');
set(gca,'xtick',[]);
set(gca,'ytick',[]);
subtitles = {'training demonstration','holdout demonstration #%i','learned reward optimal path #%i'};
subtitles2 = {'(1 of 16 examples)','(not part of the training set)',''};
labels = {'training demonstration','holdout demonstration','learned reward path'};
labelcolors = {[1.0 0.9 0.7],[0.7 1.0 0.7],[0.7 0.7 1.0]};
carcolors = {[0.8 0.7 0.0],[0.1 0.8 0.3],[0.0 0.3 1.0]};

frame = 1;

% Render main title slide.
set(gca,'xlim',[-FOLLOWHSIZE FOLLOWHSIZE]);
set(gca,'ylim',[-FOLLOWVSIZE FOLLOWVSIZE]);
rectangle('position',[-FOLLOWHSIZE-10 -FOLLOWVSIZE-10 (FOLLOWHSIZE+10)*2 (FOLLOWVSIZE+10)*2],'facecolor','k');
text(0,0.38,'Continuous Inverse','horizontalalignment','center','color','w','FontSize',44);
text(0,0.2,'Optimal Control with Locally','horizontalalignment','center','color','w','FontSize',44);
text(0,0.02,'Optimal Examples','horizontalalignment','center','color','w','FontSize',44);
text(0,-0.25,'driving policies learned','horizontalalignment','center','color','w','FontSize',34);
text(0,-0.39,'from human examples','horizontalalignment','center','color','w','FontSize',34);
% Render the frame.
for t=1:TITLE_FRAMES,
    pause(0.01);
    I = getframe(gcf,[0 0 w h]);
    I = imresize(I.cdata,0.5);
    if SAVE,
        imwrite(I,[outpath 'frame' num2str(frame) '.png'],'png');
    end;
    frame = frame + 1;
    pause(0.01);
end;
        
% Step over each example.
for i=1:length(files),
    % Load the mat file.
    load(files{i});
    evalstruct = highwayevalprepare(test_result.mdp_params);

    % Draw all videos.
    for j=1:length(example_idx{i}),
        % Render title slide.
        set(gca,'xlim',[-FOLLOWHSIZE FOLLOWHSIZE]);
        set(gca,'ylim',[-FOLLOWVSIZE FOLLOWVSIZE]);
        rectangle('position',[-FOLLOWHSIZE-10 -FOLLOWVSIZE-10 (FOLLOWHSIZE+10)*2 (FOLLOWVSIZE+10)*2],'facecolor','k');
        
        % Title.
        type = example_type{i}{j};
        text(0,0.1,names{i},'horizontalalignment','center','color','w','FontSize',44);
        
        % Subtitle.
        if type == 1,
            text(0,-0.1,subtitles{type},'horizontalalignment','center','color','w','FontSize',34);
        else
            text(0,-0.1,sprintf(subtitles{type},cnt{i}{j}),'horizontalalignment','center','color','w','FontSize',34);
        end;
        text(0,-0.22,subtitles2{type},'horizontalalignment','center','color','w','FontSize',30);
        
        % Render the frame.
        for t=1:TITLE_FRAMES,
            pause(0.01);
            I = getframe(gcf,[0 0 w h]);
            I = imresize(I.cdata,0.5);
            if SAVE,
                imwrite(I,[outpath 'frame' num2str(frame) '.png'],'png');
            end;
            frame = frame + 1;
            pause(0.01);
        end;
        clf;
        hold on;
        gca;
        set(gca,'position',[0 0 1 1]);
        set(gca,'color','none','xgrid','off','ygrid','off','visible','off');
        set(gca,'xtick',[]);
        set(gca,'ytick',[]);

        % Draw first frame.
        mdp_data = test_result.mdp_data;
        mdp_params = test_result.mdp_params;
        mdp_params = highwaydefaultparams(mdp_params);
        mdp = 'highway';
        if type==1,
            example = test_result.example_samples{example_idx{i}{j}};
        elseif type == 2,
            example = test_result.test_samples{example_idx{i}{j}};
        else
            example = test_result.irl_result.test_samples{example_idx{i}{j}};
        end;
        drawstruct = highwaydrawcreate(mdp_data,mdp_params,2);
        drawstruct = highwaydrawmodify(drawstruct,mdp_data,mdp_params,0,2);
        T = size(example.u,1);
        x = feval(strcat(mdp,'control'),mdp_data,example.s,example.u);
        TEXTX = 0.93;
        TEXTY = 0.94;
        speed = text(TEXTX,TEXTY,[num2str(x(1,4)*1000,'%3.1f') ' kph'],...
            'horizontalalignment','right','FontSize',40,'color',labelcolors{type},'units','normalized');
        highlightrect = rectangle('position',[x(1,1)-0.1 x(1,2)-0.1 0.2 0.2],'linewidth',3);
        text(0.02,0.95,labels{type},'FontSize',34,'color',labelcolors{type},'units','normalized');
        text(0.02,0.06,lower(names{i}),'FontSize',34,'color',labelcolors{type},'units','normalized');
        cutoff = text(0.9,0.88,'cutting off','horizontalalignment','right','FontSize',30,...
            'color',[1.0 0.7 0.7],'units','normalized','visible','off');
        tailgate = text(0.9,0.83,'tailgating','horizontalalignment','right','FontSize',30,...
            'color',[0.9 1.0 0.6],'units','normalized','visible','off');

        % Step over each frame and render it into a png file.
        for t=1:(T-CUTOFFTAIL),
            % Get position and update view.
            pos = x(t,1:2);
            theta = x(t,3);
            set(gca,'xlim',[pos(1)-FOLLOWHSIZE pos(1)+FOLLOWHSIZE]);
            set(gca,'ylim',[pos(2)-FOLLOWVSIZE pos(2)+FOLLOWVSIZE]);
            
            % Check for tailgating and cutoff.
            %tg = highwayevalcompute(evalstruct,x(t,:),t,mdp_data);
            
            % Set tailgating as visible.
            %if tg(3) > 0,
            %    set(tailgate,'visible','on');
            %else
            %    set(tailgate,'visible','off');
            %end;
            % Set cutoff as visible.
            %if tg(4) > 0,
            %    set(cutoff,'visible','on');
            %else
            %    set(cutoff,'visible','off');
            %end;
            
            % Set speed test.
            set(speed,'string',[num2str(x(t,4)*1000,'%3.1f') ' kph']);
            
            % Render all cars.
            f = max(0,min(1,(t-13)/2));
            drawstruct = highwaydrawmodify(drawstruct,mdp_data,mdp_params,t,2-f);
            
            % Now render the example.
            [xv,yv] = highwaycarplot(pos,theta,drawstruct.car_width,drawstruct.car_height);
            col = carcolors{type};
            if t > 1,
                set(explot,'xdata',xv);
                set(explot,'ydata',yv);
            else
                explot = patch(xv,yv,col,'edgecolor','k');
            end;
            
            % Set transparency on arrow.
            [xv,yv] = highwayarrowplot(pos,theta,drawstruct.car_width,drawstruct.car_height);
            col = labelcolors{type};
            if t > 1,
                set(exaplot,'xdata',xv);
                set(exaplot,'ydata',yv);
            else
                exaplot = patch(xv,yv,col*t,'edgecolor','none');
            end;
            if f >= 1,
                set(exaplot,'facecolor','none');
            else
                set(exaplot,'facecolor',col*(1-f) + [0.2 0.2 0.2]*f);
            end;
            
            % Put a box around the example if necessary.
            if mod(ceil(t/2),2) == 1 && t < 15,
                set(highlightrect,'position',[pos(1)-0.08 pos(2)-0.08 0.16 0.16]);
                set(highlightrect,'edgecolor',labelcolors{type});
            else
                set(highlightrect,'edgecolor','none');
            end;
            
            % Store the frame.
            pause(0.01);
            I = getframe(gcf,[0 0 w h]);
            I = imresize(I.cdata,0.5);
            if SAVE,
                imwrite(I,[outpath 'frame' num2str(frame) '.png'],'png');
            end;
            frame = frame + 1;
            pause(0.01);
        end;
    end;
end;
