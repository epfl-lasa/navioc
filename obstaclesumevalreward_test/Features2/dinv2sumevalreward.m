function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	dinv2sumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

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
		if ((reward.skip_other && mdp_data.other_pair(i, j)) || ...
			(reward.skip_wp && mdp_data.wheelchair_pedestrian_pair(i, j)) || ...
			(reward.skip_wc && mdp_data.wheelchair_companion_pair(i, j)))
			continue
		end
		% assume that ix1 < ix2
		ip1 = (i*2 - 1):(i*2);
		ip2 = (j*2 - 1):(j*2);

		P = states(:, ip1) - states(:, ip2);
		P2 = sum(P.^2, 2);

		P2inv = 1./P2;

		r = r + P2inv;

		if nargout >= 2
			k = -2*P2inv.^2;

			drdp1 = k.*P;

			drdx(:, ip1) = drdx(:, ip1) + drdp1;
			drdx(:, ip2) = drdx(:, ip2) - drdp1;
		end

		if nargout >= 6
			d2rdp1dp1 = k.*(reshape(eye(2), [1, 2, 2]) - 4*P2inv.*P.*reshape(P, [Nt, 1, 2]));

			d2rdxdx(:, ip1, ip1) = d2rdxdx(:, ip1, ip1) + d2rdp1dp1;
			d2rdxdx(:, ip1, ip2) = d2rdxdx(:, ip1, ip2) - d2rdp1dp1;
			d2rdxdx(:, ip2, ip1) = d2rdxdx(:, ip2, ip1) - d2rdp1dp1;
			d2rdxdx(:, ip2, ip2) = d2rdxdx(:, ip2, ip2) + d2rdp1dp1;
		end
	end
end

normalizer = n_agents*reward.expec; % n_agents*(n_agents - 1)/2;
r = r/normalizer;
if nargout >= 2
	drdx = drdx/normalizer;
	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end
if nargout >= 6
	d2rdxdx = d2rdxdx/normalizer;
    %disp(reward.type)
end