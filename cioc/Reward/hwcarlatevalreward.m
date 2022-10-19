% Highway car perpendicular distance feature.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    hwcarlatevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

% Check if this is a visualization call.
if isempty(x) && size(states,2) == 3,
    % This visualization call is in Cartesian coordinates.
    states = [states(:,1:2) zeros(size(states,1),3) states(:,3)];
end;

% Get constants.
T = size(u,1);
Du = size(u,2);
Dx = size(states,2);

if nargout > 1,
    [r1,~,dr1dx,~] = highwaycareval(states,reward);
else
    [r1,~] = highwaycareval(states,reward);
end;

% Compute value.
r = reward.r*r1;

if nargout >= 2,
    % Compute distance gradients.
    % Note that the gradient with respect to time is not actually zero.
    % However, since no action actually influences time, these dimensions
    % are removed completely when we multiply by the Jacobian, so we do not
    % need to compute them.
    drdx = [reward.r*dr1dx zeros(T,4)];
    
    if isfield(mdp_data,'defergrad'),
        g = {drdx};
    else
        g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
    end;
end;

if nargout >= 3,
    % Gradients with respect to controls are always zero.
    drdu = zeros(T,Du);
    d2rdudu = zeros(T,Du,Du);
end;

if nargout >= 6,
    % Hessian is zero.
    d2rdxdx = zeros(T,Dx,Dx);
end;

if nargout >= 7,
    % Compute gfull.
    % Convert gradient to T x TD matrix.
    drdxmat = zeros(T,T*Dx);
    for i=1:Dx,
        drdxmat(:,(i-1)*T + (1:T)) = diag(drdx(:,i));
    end;

    % Compute gradient with respect to controls.
    gfull = drdxmat * dxdu';
    
    % Compute Hfull.
    Hfull = zeros(T,T*Du,T*Du);
    for t=1:T,
        idxs = (0:(Dx-1))*T + t;
        D = permute(d2rdxdx(t,:,:),[2 3 1]);
        Hfull(t,:,:) = dxdu(:,idxs) * D * dxdu(:,idxs)' + sum(bsxfun(@times,permute(drdx(t,:),[1 3 2]),d2xdudu(:,:,idxs)),3);
    end;
end;
