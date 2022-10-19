% Evaluate controls on the robot arm and return gradients and Hessians.
function [states,A,B,invB,dxdu,d2xdudu] = robotarmcontrol(mdp_data,x,u)

% Constants.
Dx = mdp_data.dims;
Du = mdp_data.udims;
T = size(u,1);

% Go step by step and evaluate equations of motion.
% TODO: implement the equations of motion here so that we can have a proper
% dynamic robot arm
%{
B = cell(1,T);
xc = x;
for t=1:T,
    % Compute forward kinematics.
    [ptx,pty,jacx,jacy,jjacx,jjacy] = robotarmfk(xc,mdp_data);
    
    % Compute diagonal mass.
    M = sum(bsxfun(@times,bsxfun(@minus,ptx,ptx').^2 + bsxfun(@minus,pty,pty').^2,mdp_data.linkmass),2);
    
    % Compute inertial term.
end;
%}

% Allocate space.
states = zeros(T,Dx);

% Integrate velocities.
states(:,(Du+1):Dx) = bsxfun(@plus,cumsum(bsxfun(@rdivide,u,mdp_data.linkmass),1),x(1,(Du+1):Dx));

% Integrate positions.
states(:,1:Du) = bsxfun(@plus,cumsum(states(:,(Du+1):Dx),1),x(1,1:Du));

% Put positions in range -pi to pi.
states(:,1:Du) = mod(states(:,1:Du)+pi,2.0*pi)-pi;

% Now compute the Jacobian.
if nargout > 1,
    % First, compute A and B matrices.
    A = zeros(Dx,Dx,T);
    B = zeros(Dx,Du,T);
    invB = zeros(Du,Dx,T);
    for t=1:T,
        A(:,:,t) = [eye(Du) eye(Du); zeros(Du,Du) eye(Du)];
        B(:,:,t) = bsxfun(@rdivide,[eye(Du); eye(Du)],mdp_data.linkmass);
        invB(:,:,t) = pinv(B(:,:,t));
    end;
    
    if nargout >= 5,
        % Now build the Jacobian out of these matrices.
        dxdu = zeros(Du*T,Dx*T);
        uidx = (0:T:(T*(Du-1)));
        xidx = (0:T:(T*(Dx-1)));
        for c=1:T,
            % First, compute the top part of this block row.
            for r=1:(c-1),
                dxdu(r + uidx, c + xidx) = dxdu(r + uidx, (c-1) + xidx)*A(:,:,c)';
            end;
            % Now write the diagonal block.
            dxdu(c + uidx, c + xidx) = B(:,:,c)';
        end;
    end;
end;

% The Hessian is zero.
% Note that this is only strictly true in the kinematic case. In the
% inertial case, this is not true, but we can use it as an approximation in
% order to keep using the linear-time algorithm.
if nargout >= 6,
    d2xdudu = zeros(Du*T,Du*T,Dx*T);
end;
