function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	vmagerr2sumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

Nt = size(u, 1);
Nu = size(u, 2);
Nx = size(states, 2);

if reward.type_other
    idx = mdp_data.idx_other;
elseif reward.type_w
    idx = mdp_data.idx_wheelchair;
elseif reward.type_c
    idx = mdp_data.idx_companion;
else
    idx = 1:(Nu/2);
end

vmag_des = mdp_data.vmag_des(idx);

n = length(idx);
jj_vx = Nu + (idx*2 - 1);
jj_vy = Nu + idx*2;

Vx = states(:, jj_vx);
Vy = states(:, jj_vy);

V2 = Vx.^2 + Vy.^2;
Vmag = sqrt(V2);

r = 0.5*sum((Vmag - vmag_des).^2, 2);

if nargout >= 2
	drdx = zeros(Nt, Nx);
	k = 1 - vmag_des./Vmag;
	drdx(:, jj_vx) = k.*Vx;
	drdx(:, jj_vy) = k.*Vy;
	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end
if nargout >= 3
	drdu = zeros(Nt, Nu);
	d2rdudu = zeros(Nt, Nu, Nu);
end
if nargout >= 6
	d2rdxdx = zeros(Nt, Nx, Nx);
	mask = repmat(reshape(eye(n), [1, n, n]), [Nt, 1, 1]);
	l = vmag_des./V2./Vmag;
	d2rdxdx(:, jj_vx, jj_vx) = (k + Vx.^2.*l).*mask;
	d2rdxdx(:, jj_vx, jj_vy) = Vx.*Vy.*l.*mask;
	d2rdxdx(:, jj_vy, jj_vx) = d2rdxdx(:, jj_vx, jj_vy);
	d2rdxdx(:, jj_vy, jj_vy) = (k + Vy.^2.*l).*mask;
end