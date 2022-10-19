function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	xerr2sumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

Nt = size(u, 1);
Nu = size(u, 2);
Nx = size(states, 2);

idx = (1:Nu) + Nu*reward.order;
err = states(:, idx) - reward.x_des;
r = 0.5*sum(err.^2, 2);
if nargout >= 2
	drdx = zeros(Nt, Nx);
	drdx(:, idx) = err;
	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end
if nargout >= 3
	drdu = zeros(Nt, Nu);
	d2rdudu = zeros(Nt, Nu, Nu);
end
if nargout >= 6
	d2rdxdx = zeros(Nt, Nx, Nx);
	d2rdxdx(:, idx, idx) = repmat(reshape(eye(Nu), 1, Nu, Nu), Nt);
end