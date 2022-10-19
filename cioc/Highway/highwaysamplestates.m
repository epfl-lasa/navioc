% Sample some random states in the Highway environment.
function s = highwaysamplestates(n,mdp_data)

SPEED = 0.05;

s = zeros(n,6);
for i=1:n,
    % Pick a lane.
    lane = randi(length(mdp_data.lanes));
    mxs = mdp_data.lanes{lane}.xs;
    mxe = mdp_data.lanes{lane}.xe;
    mys = mdp_data.lanes{lane}.ys;
    mye = mdp_data.lanes{lane}.ye;
    
    % Add up the total length of all straightaways.
    xl = sqrt(sum((mxe - mxs).^2,2));
    yl = sqrt(sum((mye - mys).^2,2));
    tl = sum(xl) + sum(yl);
    pos = rand(1,1)*tl;
    xs = cumsum(xl);
    ys = cumsum(yl) + sum(xl);
    ge = find(xs > pos);
    if isempty(ge),
        ge = find(ys > pos);
        ge = ge(1);
        pos = (pos-ys(ge)+yl(ge))/yl(ge);
        pos = mys(ge,:)*pos + mye(ge,:)*(1-pos);
        if mys(ge,2) < mye(ge,2),
            heading = 0;
        else
            heading = pi;
        end;
    else
        ge = ge(1);
        pos = (pos-xs(ge)+xl(ge))/xl(ge);
        pos = mxs(ge,:)*pos + mxe(ge,:)*(1-pos);
        if mxs(ge,1) > mxe(ge,1),
            heading = 3*pi/2;
        else
            heading = pi/2;
        end;
    end;

    s(i,:) = [pos heading SPEED 0 0];
end;
