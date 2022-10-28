function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	dgaussiansumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

Nt = size(u, 1);
Nu = size(u, 2);
Nx = size(states, 2);

% initialize sums to zero
r = zeros(Nt, 1);
if nargout >= 2
	drdx = zeros(Nt, Nx);
end
if nargout >= 3
	drdu = zeros(Nt, Nu);
	d2rdudu = zeros(Nt, Nu, Nu);
end
if nargout >= 6
	d2rdxdx = zeros(Nt, Nx, Nx);
end

% iterate over pairs of agents
n_agents = Nu/2;
for i = 1:n_agents
	for j = (i + 1):n_agents
		% assume that ix1 < ix2
		ip1 = (i*2 - 1):(i*2);
		ip2 = (j*2 - 1):(j*2);

		P = states(:, ip1) - states(:, ip2);
		P2 = sum(P.^2, 2);

		f = exp(-0.5*P2/reward.sigma^2);

		r = r + f;

		if nargout >= 2
			k = -f/reward.sigma^2;

			dfdp1 = k.*P;

			drdx(:, ip1) = drdx(:, ip1) + dfdp1;
			drdx(:, ip2) = drdx(:, ip2) - dfdp1;
		end

		if nargout >= 6
			d2fdp1dp1 = k.*(reshape(eye(2), [1, 2, 2]) - (1/reward.sigma^2)*P.*reshape(P, [Nt, 1, 2]));

			d2rdxdx(:, ip1, ip1) = d2rdxdx(:, ip1, ip1) + d2fdp1dp1;
			d2rdxdx(:, ip1, ip2) = d2rdxdx(:, ip1, ip2) - d2fdp1dp1;
			d2rdxdx(:, ip2, ip1) = d2rdxdx(:, ip2, ip1) - d2fdp1dp1;
			d2rdxdx(:, ip2, ip2) = d2rdxdx(:, ip2, ip2) + d2fdp1dp1;
		end
	end
end

if nargout >= 2
	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end