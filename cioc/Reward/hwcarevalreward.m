% Highway car Gaussian reward feature.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    hwcarevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

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
    [r1,r2,dr1dx,dr2dx] = highwaycareval(states,reward);
else
    [r1,r2] = highwaycareval(states,reward);
end;

% Compute distance.
d = reward.lam(1)*(r1.^2) + reward.lam(2)*(r2.^2);

% Compute value.
r = reward.r*exp(-0.5*reward.width*d);

if nargout >= 2,
    % Compute distance gradients.
    dp = 2.0*reward.lam(1)*bsxfun(@times,r1,dr1dx) + 2.0*reward.lam(2)*bsxfun(@times,r2,dr2dx);

    % Compute overall gradient.
    % Note that the gradient with respect to time is not actually zero.
    % However, since no action actually influences time, these dimensions
    % are removed completely when we multiply by the Jacobian, so we do not
    % need to compute them.
    dg = [dp zeros(T,4)];

    % Compute gradient.
    drdx = bsxfun(@times,-0.5*reward.width*r,dg);
    
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
    % Compute point second derivative matrix.
    % Note that the Hessian with respect to time is not actually zero.
    % However, since no action actually influences time, these dimensions
    % are removed completely when we multiply by the Jacobian, so we do not
    % need to compute them.
    ddgxx = 2*reward.lam(1).*(dr1dx(:,1).^2) + 2.0*reward.lam(2).*(dr2dx(:,1).^2);
    ddgyy = 2*reward.lam(1).*(dr1dx(:,2).^2) + 2.0*reward.lam(2).*(dr2dx(:,2).^2);
    ddgxy = 2*(dr2dx(:,1).*dr2dx(:,2)*reward.lam(2) + dr1dx(:,1).*dr1dx(:,2)*reward.lam(1));
    
    % drdx has already been computed.
    d2rdxdx = zeros(T,Dx,Dx);
    for t=1:T,
        % Construct first part.
        d2rdp2 = (r(t)*(reward.width^2)/4)*dg(t,:)'*dg(t,:);
        
        % Construct the second part.
        d2rdp2 = d2rdp2 - (r(t)*reward.width/2)*[ddgxx(t) ddgxy(t) 0 0 0 0;...
                                                 ddgxy(t) ddgyy(t) 0 0 0 0;...
                                                 0 0 0 0 0 0;...
                                                 0 0 0 0 0 0;...
                                                 0 0 0 0 0 0;...
                                                 0 0 0 0 0 0];
        
        % Construct the Hessian.
        d2rdxdx(t,:,:) = d2rdp2;
    end;
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
