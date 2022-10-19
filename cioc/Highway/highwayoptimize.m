% Stepwise optimization of a single car's trajectory.
function [u,r] = highwayoptimize(x0,u0,mdp_data,mdp,reward,options)

% Constants.
STEP = 32;
HSTEP = 8;
T = size(u0,1);

% Suppress warning:
warning off MATLAB:nearlySingularMatrix;

% First, run a standard optimization to get a baseline.
[u1,r1] = minFunc(@(p)trajectoryreward(p,x0,mdp_data,mdp,reward),u0(:),options);
u1 = reshape(u1,T,mdp_data.udims);

% Now do a stepwise optimization in steps of 16.
u2 = u0;
states = zeros(T,mdp_data.dims);
for i=1:HSTEP:T,
    % First optimize the tail seperately.
    idx = i:min(T,(i-1+STEP));
    u0step = u2(idx,:);
    if i == 1,
        x0step = x0;
    else
        x0step = states(i-1,:);
    end;
    ustep = minFunc(@(p)trajectoryreward(p,x0step,mdp_data,mdp,reward),u0step(:),options);
    ustep = reshape(ustep,length(idx),mdp_data.udims);
    u2(idx,:) = ustep;
    states(idx,:) = feval(strcat(mdp,'control'),mdp_data,x0step,ustep);
    
    %{
    % Now optimize the whole thing.
    idxall = 1:min(T,(i-1+STEP));
    u0step = u2(idxall,:);
    ustep = minFunc(@(p)trajectoryreward(p,x0,mdp_data,mdp,reward),u0step(:),options);
    ustep = reshape(ustep,length(idxall),mdp_data.udims);
    states(idxall,:) = feval(strcat(mdp,'control'),mdp_data,x0,ustep);
    u2(idxall,:) = ustep;
    %}
end;
[u2,r2] = minFunc(@(p)trajectoryreward(p,x0,mdp_data,mdp,reward),u2(:),options);

% Turn on warning:
warning on MATLAB:nearlySingularMatrix;

% Pick the best of the two.
if r1 < r2,
    u = u1;
    r = r1;
    %fprintf(1,'Picking original result: %f vs %f\n',r1,r2);
else
    u = u2;
    r = r2;
    %fprintf(1,'Picking stepwise result: %f vs %f\n',r2,r1);
end;
