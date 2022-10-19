% Perform forward kinematics to determine Cartesian positions of end
% effectors.
function [ptx,pty,jacx,jacy,jjacx,jjacy] = robotarmfk(states,mdp_data)

% Run forward kinematics.
T = size(states,1);
N = mdp_data.links;
ptx = zeros(T,N);
pty = zeros(T,N);
ptx0 = 0.5*ones(T,1)*mdp_data.bounds(1);
pty0 = 0.5*ones(T,1)*mdp_data.bounds(2);
ptxp = ptx0;
ptyp = pty0;
a = zeros(T,1);
for i=1:N,
    % Increment angles.
    a = a + states(:,i);
    
    % Compute positions.
    ptx(:,i) = ptxp + mdp_data.linklen(i)*sin(a);
    pty(:,i) = ptyp + mdp_data.linklen(i)*cos(a);
    
    % Increment running positions.
    ptxp = ptx(:,i);
    ptyp = pty(:,i);
end;

% Compute Jacobians.
if nargout > 2,
    % These are preceding (pivot) points.
    fpx = [ptx0 ptx(:,1:(N-1))];
    fpy = [pty0 pty(:,1:(N-1))];
    
    % Compute full Jacobians (without zero entries).
    jacx = bsxfun(@minus,pty,permute(fpy,[1 3 2]));
    jacy = bsxfun(@plus,-ptx,permute(fpx,[1 3 2]));
    
    % Now zero out those entries where i_3 > i_2.
    mask = permute(tril(ones(N,N)),[3 1 2]);
    jacx = bsxfun(@times,jacx,mask);
    jacy = bsxfun(@times,jacy,mask);
end;

% Compute second derivatives.
if nargout > 4,
    % Construct index matrix.
    idx = tril(repmat((1:N)',1,N)) + triu(repmat(1:N,N,1)) - diag(1:N);
    jjacx = reshape(jacy(:,:,idx),[T N N N]);
    jjacy = reshape(-jacx(:,:,idx),[T N N N]);
end;
