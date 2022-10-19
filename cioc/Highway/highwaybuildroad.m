% Build road structure from points.
function feature = highwaybuildroad(pts)

% Convert to lines and circles.
EPS = 1.0e-8;
linesx = struct('start',[],'end',[]);
linesy = struct('start',[],'end',[]);
circles = struct('center',[],'radius',[],'quarter',[]);
pts = [pts; pts(1:2,:)];
cx = 1;
cy = 1;
cc = 1;
for i=1:(size(pts,1)-2),
    % Check if this segment is a line or a circle.
    if pts(i,1) == pts(i+1,1),
        % This is a vertical line.
        linesy(cy) = struct('start',pts(i,:),'end',pts(i+1,:));
        cy = cy+1;
    elseif pts(i,2) == pts(i+1,2),
        % This is a horizontal line.
        linesx(cx) = struct('start',pts(i,:),'end',pts(i+1,:));
        cx = cx+1;
    else
        % This is a circle.
        % Figure out which quarter this is.
        if pts(i+1,1) == pts(i+2,1) || pts(i+1,2) == pts(i+2,2),
            % Next segment is a line.
            if pts(i+1,1) == pts(i+2,1),
                % Vertical line.
                if pts(i+2,2) > pts(i+1,2) && pts(i,1) < pts(i+1,1),
                    q = 2;
                elseif pts(i+2,2) > pts(i+1,2) && pts(i,1) > pts(i+1,1),
                    q = 3;
                elseif pts(i+2,2) < pts(i+1,2) && pts(i,1) > pts(i+1,1),
                    q = 4;
                elseif pts(i+2,2) < pts(i+1,2) && pts(i,1) < pts(i+1,1),
                    q = 1;
                end;
                c = [pts(i,1) pts(i+1,2)];
            else
                % Horizontal line.
                if pts(i+2,1) > pts(i+1,1) && pts(i,2) < pts(i+1,2),
                    q = 4;
                elseif pts(i+2,1) > pts(i+1,1) && pts(i,2) > pts(i+1,2),
                    q = 3;
                elseif pts(i+2,1) < pts(i+1,1) && pts(i,2) > pts(i+1,2),
                    q = 2;
                elseif pts(i+2,1) < pts(i+1,1) && pts(i,2) < pts(i+1,2),
                    q = 1;
                end;
                c = [pts(i+1,1) pts(i,2)];
            end;
        else
            % Next segment is a circle.
            % Average opposite points to get center (assumes equal
            % radius).
            c = (pts(i+2,:)+pts(i,:))*0.5;
            if pts(i,2) > c(2)+EPS || pts(i+1,2) > c(2)+EPS,
                if pts(i,1) > c(1)+EPS || pts(i+1,1) > c(1)+EPS,
                    q = 1;
                else
                    q = 4;
                end;
            else
                if pts(i,1) > c(1)+EPS || pts(i+1,1) > c(1)+EPS,
                    q = 2;
                else
                    q = 3;
                end;
            end;
        end;
        r = norm(pts(i,:)-c);
        circles(cc) = struct('center',c,'radius',r,'quarter',q);
        cc = cc+1;
    end;
end;

% Create course feature.
feature = struct('type','hwlane','width',1000.0,'r',1.0,...
    'xs',vertcat(linesx.start),'xe',vertcat(linesx.end),...
    'ys',vertcat(linesy.start),'ye',vertcat(linesy.end),...
    'cc',vertcat(circles.center),'cr',vertcat(circles.radius),...
    'cq',vertcat(circles.quarter));
