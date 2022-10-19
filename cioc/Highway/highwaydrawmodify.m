% Modify highway drawing structure based on the current frame.
function drawstruct = highwaydrawmodify(drawstruct,mdp_data,mdp_params,t,fill)

if nargin < 5,
    fill = 1;
end;

% Render all cars.
for i=1:length(mdp_data.cars),
    if t == 0,
        pos = mdp_data.cars{i}.x0(1,1:2);
        theta = mdp_data.cars{i}.x0(1,3);
    else
        T = min(t,size(mdp_data.cars{i}.x,1));
        pos = mdp_data.cars{i}.x(T,1:2);
        theta = mdp_data.cars{i}.x(T,3);
    end;
    [xv,yv] = highwaycarplot(pos,theta,drawstruct.car_width,drawstruct.car_height);
    [axv,ayv] = highwayarrowplot(pos,theta,drawstruct.car_width,drawstruct.car_height);
    if t == 0,
        drawstruct.cara_handles(i) = patch(axv,ayv,drawstruct.cara_color,'edgecolor','none');
        if fill <= 1,
            set(drawstruct.cara_handles(i),'facecolor','none');
        end;
        if fill,
            drawstruct.car_handles(i) = patch(xv,yv,drawstruct.car_color,'edgecolor','k');
        else
            drawstruct.car_handles(i) = patch(xv,yv,drawstruct.car_color,'edgecolor','k','facecolor','none','linewidth',3);
        end;
    else
        if fill <= 1,
            set(drawstruct.cara_handles(i),'facecolor','none');
        elseif fill > 1,
            f = 2-fill;
            set(drawstruct.cara_handles(i),'xdata',axv);
            set(drawstruct.cara_handles(i),'ydata',ayv);
            set(drawstruct.cara_handles(i),'facecolor',drawstruct.cara_color*(1-f)+[0.2 0.2 0.2]*f);
        end;
        set(drawstruct.car_handles(i),'xdata',xv);
        set(drawstruct.car_handles(i),'ydata',yv);
    end;
end;
