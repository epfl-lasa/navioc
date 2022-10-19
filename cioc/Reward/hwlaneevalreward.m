% Highway lane reward feature.
function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    hwlaneevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

% Check if this is a visualization call.
if isempty(x) && size(states,2) == 3,
    % This visualization call is in Cartesian coordinates.
    states = [states(:,1:2) zeros(size(states,1),4)];
end;

% Get constants.
T = size(u,1);
Du = size(u,2);
Dx = size(states,2);

% Compute distances.
% Get points.
pts = states(:,1:2);

% Build arrays.
xs = reward.xs;
xe = reward.xe;
ys = reward.ys;
ye = reward.ye;
cc = reward.cc;
cr = reward.cr;
cq = reward.cq;

% Evaluate distance functions for each sample.
dxl = bsxfun(@minus,pts(:,2),xs(:,2)').^2;
dyl = bsxfun(@minus,pts(:,1),ys(:,1)').^2;
rcc = sqrt(sum(bsxfun(@minus,permute(pts,[1 3 2]),permute(cc,[3 1 2])).^2,3));
dcc = bsxfun(@minus,rcc,cr').^2;

% Evaluate gradients for each sample.
gxxl = zeros(size(dxl));
gyxl = 2.0*bsxfun(@minus,pts(:,2),xs(:,2)');
gxyl = 2.0*bsxfun(@minus,pts(:,1),ys(:,1)');
gyyl = zeros(size(dyl));
grcc = 2.0*bsxfun(@rdivide,bsxfun(@minus,rcc,cr'),max(rcc,1.0e-8));
gxcc = bsxfun(@minus,pts(:,1),cc(:,1)').*grcc;
gycc = bsxfun(@minus,pts(:,2),cc(:,2)').*grcc;

% Throw out distances for lines that are past the ends.
dxl(bsxfun(@lt,pts(:,1),min(xs(:,1),xe(:,1))')) = Inf;
dxl(bsxfun(@gt,pts(:,1),max(xs(:,1),xe(:,1))')) = Inf;
dyl(bsxfun(@lt,pts(:,2),min(ys(:,2),ye(:,2))')) = Inf;
dyl(bsxfun(@gt,pts(:,2),max(ys(:,2),ye(:,2))')) = Inf;

% Throw out distances for circles that are in the wrong quarter.
a = atan2(-bsxfun(@minus,pts(:,1),cc(:,1)'),-bsxfun(@minus,pts(:,2),cc(:,2)'));
a = a + pi;
q = (a >= 0 & a < pi/2)*1 + (a >= pi/2 & a < pi)*2 + (a >= pi & a < 3*pi/2)*3 + (a >= 3*pi/2 & a < 2*pi)*4;
dcc(q ~= repmat(cq',size(pts,1),1)) = Inf;

% Now take the minimum distance in each bunch.
[dxl,idxl] = min(dxl,[],2);
idxl = sub2ind(size(gxxl),(1:size(pts,1))',idxl);
[dyl,idyl] = min(dyl,[],2);
idyl = sub2ind(size(gxyl),(1:size(pts,1))',idyl);
[dcc,idcc] = min(dcc,[],2);
idcc = sub2ind(size(gxcc),(1:size(pts,1))',idcc);
gx = gxxl(idxl).*(dxl <= dyl & dxl <= dcc) + gxyl(idyl).*(dyl < dxl & dyl <= dcc) + gxcc(idcc).*(dcc < dxl & dcc < dyl);
gy = gyxl(idxl).*(dxl <= dyl & dxl <= dcc) + gyyl(idyl).*(dyl < dxl & dyl <= dcc) + gycc(idcc).*(dcc < dxl & dcc < dyl);
d = min(min(dxl,dyl),dcc);

% Compute value.
r = reward.r*exp(-0.5*reward.width*d);

if nargout >= 2,
    % Compute gradient.
    drdx = [-0.5*reward.width*r.*gx, -0.5*reward.width*r.*gy, zeros(T,4)];
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
    % Evaluate second derivatives for each sample.
    gxxxl = zeros(T,size(xs,1));
    gyyxl = 2.0*ones(T,size(xs,1));
    gxyxl = zeros(T,size(xs,1));
    gxxyl = 2.0*ones(T,size(ys,1));
    gyyyl = zeros(T,size(ys,1));
    gxyyl = zeros(T,size(ys,1));
    gxxcc = 2.0*(bsxfun(@minus,pts(:,1),cc(:,1)').^2).*bsxfun(@rdivide,cr',max(rcc.^3,1.0e-8)) + grcc;
    gyycc = 2.0*(bsxfun(@minus,pts(:,2),cc(:,2)').^2).*bsxfun(@rdivide,cr',max(rcc.^3,1.0e-8)) + grcc;
    gxycc = 2.0*bsxfun(@minus,pts(:,1),cc(:,1)').*bsxfun(@minus,pts(:,2),cc(:,2)').*bsxfun(@rdivide,cr',max(rcc.^3,1.0e-8));

    gxx = gxxxl(idxl).*(dxl <= dyl & dxl <= dcc) + gxxyl(idyl).*(dyl < dxl & dyl <= dcc) + gxxcc(idcc).*(dcc < dxl & dcc < dyl);
    gyy = gyyxl(idxl).*(dxl <= dyl & dxl <= dcc) + gyyyl(idyl).*(dyl < dxl & dyl <= dcc) + gyycc(idcc).*(dcc < dxl & dcc < dyl);
    gxy = gxyxl(idxl).*(dxl <= dyl & dxl <= dcc) + gxyyl(idyl).*(dyl < dxl & dyl <= dcc) + gxycc(idcc).*(dcc < dxl & dcc < dyl);

    % drdx has already been computed.
    d2rdxdx = zeros(T,Dx,Dx);
    for t=1:T,
        % Construct first part.
        d2rdp2 = (r(t)*(reward.width^2)/4)*[gx(t)*gx(t) gx(t)*gy(t); gx(t)*gy(t) gy(t)*gy(t)];
        
        % Construct the second part.
        d2rdp2 = d2rdp2 - (r(t)*reward.width/2)*[gxx(t) gxy(t); gxy(t) gyy(t)];
        
        % Construct the Hessian.
        d2rdxdx(t,:,:) = [eye(2); zeros(4,2)] * d2rdp2 * [eye(2) zeros(2,4)];
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
