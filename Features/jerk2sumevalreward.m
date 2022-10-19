function [r,g,drdu,d2rdudu,drdx,d2rdxdx,gfull,Hfull] = ...
    jerk2sumevalreward(reward,mdp_data,x,u,states,A,B,dxdu,d2xdudu)

Nt = size(u, 1);
Nu = size(u, 2);
Nx = size(states, 2);

r = 0.5*sum(u.^2, 2);
if nargout >= 2,
    g = u;
end;

if nargout >= 3,
    drdu = u;
    d2rdudu = repmat(reshape(eye(Nu), 1, Nu, Nu), Nt);
end;

if nargout >= 5,
    drdx = zeros(T,Dx);
    d2rdxdx = zeros(T,Dx,Dx);
end;