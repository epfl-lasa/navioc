function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	vxerr2sumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

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

n = length(idx);
jj = Nu + (idx*2 - 1);

err = states(:, jj) - mdp_data.vx_des(idx);
r = 0.5*sum(err.^2, 2);
if nargout >= 2
	drdx = zeros(Nt, Nx);
	drdx(:, jj) = err;
	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end
if nargout >= 3
	drdu = zeros(Nt, Nu);
	d2rdudu = zeros(Nt, Nu, Nu);
end
if nargout >= 6
	d2rdxdx = zeros(Nt, Nx, Nx);
	d2rdxdx(:, jj, jj) = repmat(reshape(eye(n), [1, n, n]), [Nt, 1, 1]);
end