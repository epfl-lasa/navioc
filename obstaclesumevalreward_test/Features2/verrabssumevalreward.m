function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	verrabssumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

normalizer = mdp_data.n_ped*reward.expec;

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

vx_jj = Nu + (idx*2 - 1);
vy_jj = vx_jj + 1;

Vxerr = states(:, vx_jj) - mdp_data.v_des(vx_jj - Nu);
Vyerr = states(:, vy_jj) - mdp_data.v_des(vy_jj - Nu);

s = reward.s;

Expx = exp(-2*s*Vxerr);
Expy = exp(-2*s*Vyerr);
Fx = Vxerr + 1/s*log(1 + Expx);
Fy = Vyerr + 1/s*log(1 + Expy);

r = sum(Fx + Fy, 2)/normalizer;
if nargout >= 2
	drdx = zeros(Nt, Nx);
	drdx(:, vx_jj) = (1 - 2*Expx./(1 + Expx))/normalizer;
	drdx(:, vy_jj) = (1 - 2*Expy./(1 + Expy))/normalizer;
	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end
if nargout >= 3
	drdu = zeros(Nt, Nu);
	d2rdudu = zeros(Nt, Nu, Nu);
end
if nargout >= 6
	d2rdxdx = zeros(Nt, Nx, Nx);
	n = length(idx);
	mask = reshape(eye(n), [1, n, n])/normalizer;
	d2rdxdx(:, vx_jj, vx_jj) = 4*s*Expx./(1 + Expx).^2.*mask;
	d2rdxdx(:, vy_jj, vy_jj) = 4*s*Expy./(1 + Expy).^2.*mask;
end