% Evaluate controls on the car and return gradients and Hessians.
function [states,A,B,invB,dxdu,d2xdudu] = highwaycontrol(mdp_data,x,u)

% Constants.
Dx = mdp_data.dims;
Du = mdp_data.udims;
T = size(u,1);

% These are trivial dynamics that are useful for debugging Hessians.
%{
states = bsxfun(@plus,[cumsum(u,1) zeros(T,3) (1:T)'],x);

% Now compute the Jacobian.
if nargout > 1,
    % First compute the Jacobian of points to thrust, which is block
    % triangular.
    dpdt = zeros(2*T,2*T);
    for i=1:2,
        %dpdt((i-1)*T + (1:T),(i-1)*T + (1:T)) = triu(ones(T)).*repmat(3.0*thrust(:,i)'.^2,T,1);
        dpdt((i-1)*T + (1:T),(i-1)*T + (1:T)) = triu(ones(T));
    end;
    
    % Now convert the Jacobian to take us from states to controls.
    dxdu = zeros(Du*T,Dx*T);
    for tu=1:T,
        for ts=tu:T,
            srcidxs = ts + [0 T];
            srcidxu = tu + [0 T];
            dstidxs = ts + (0:T:(T*(Dx-1)));
            dstidxu = tu + (0:T:(T*(Du-1)));
            dxdu(dstidxu,dstidxs) = dpdt(srcidxu,srcidxs) * [eye(2) zeros(2,4)];
        end;
    end;
end;

% Create A and B matrices.
if nargout > 1,
    A = zeros(Dx,Dx,T);
    B = zeros(Dx,Du,T);
    invB = zeros(Du,Dx,T);
    for t=1:T,
        dstidxs = t + (0:T:(T*(Dx-1)));
        dstidxu = t + (0:T:(T*(Du-1)));
        A(:,:,t) = eye(Dx);
        B(:,:,t) = dxdu(dstidxu,dstidxs)';
        invB(:,:,t) = pinv(B(:,:,t));
    end;
end;
%}

% Constants.
dl = mdp_data.cardamp(1);
dr = mdp_data.cardamp(2);

% Allocate space.
states = zeros(T,Dx);

% Integrate velocities.
cv = x(1,4:5);
for t=1:T,
    states(t,4:5) = cv.*mdp_data.cardamp + u(t,:)./mdp_data.carmass;
    cv = states(t,4:5);
end;

% Integrate heading.
p4 = [x(1,4); states(1:(T-1),4)];
states(:,3) = bsxfun(@plus,cumsum(states(:,5).*p4,1),x(1,3));

% Transform velocities into Cartesian coordinates.
headings = [x(1,3); states(1:(T-1),3)];
cosines = cos(headings);
sines = sin(headings);
velcart = [states(:,4).*sines states(:,4).*cosines];

% Integrate positions.
states(:,1:2) = bsxfun(@plus,cumsum(velcart,1),x(1,1:2));

% Add to time.
states(:,6) = bsxfun(@plus,(1:T)',x(1,6));

% Put heading in range -pi to pi.
states(:,3) = mod(states(:,3)+pi,2.0*pi)-pi;

% Now compute the Jacobian.
if nargout > 1,
    % Compute A and B matrices.
    A = zeros(Dx,Dx,T);
    A(1,1,:) = 1;
    A(1,3,:) = permute(cosines.*states(:,4),[3 2 1]);
    A(1,4,:) = dl*permute(sines,[3 2 1]);
    A(2,2,:) = 1;
    A(2,3,:) = permute(-sines.*states(:,4),[3 2 1]);
    A(2,4,:) = dl*permute(cosines,[3 2 1]);
    A(3,3,:) = 1;
    A(3,4,:) = permute(states(:,5),[3 2 1]);
    A(3,5,:) = dr*permute(p4(:,1),[3 2 1]);
    A(4,4,:) = dl;
    A(5,5,:) = dr;
    A(6,6,:) = 1;
    B = zeros(Dx,Du,T);
    B(1,1,:) = permute(sines,[3 2 1])/mdp_data.carmass(1);
    B(2,1,:) = permute(cosines,[3 2 1])/mdp_data.carmass(1);
    B(3,2,:) = permute(p4(:,1),[3 2 1])/mdp_data.carmass(2);
    B(4,1,:) = 1/mdp_data.carmass(1);
    B(5,2,:) = 1/mdp_data.carmass(2);
    
    % Compute inverses.
    if nargout >= 4,
        invB = zeros(Du,Dx,T);
        for t=1:T,
            invB(:,:,t) = pinv(B(:,:,t));
        end;
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
% Note that this is not actually the case, but we'll leave it like this and
% use the locally linear approximation.
if nargout >= 6,
    d2xdudu = zeros(Du*T,Du*T,Dx*T);
end;
