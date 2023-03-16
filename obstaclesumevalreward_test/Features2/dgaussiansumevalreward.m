function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	dgaussiansumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

fast = isfield(reward, 'fast') && reward.fast;

Nt = size(states, 1);
Nu = mdp_data.udims;
Nx = mdp_data.dims;

% initialize sums to zero
r = zeros(Nt, 1);
if nargout >= 2
	drdx = zeros(Nt, Nx);
end
if nargout >= 3
	if fast
		drdu = 0;
		d2rdudu = 0;
	else
		drdu = zeros(Nt, Nu);
		d2rdudu = zeros(Nt, Nu, Nu);
	end
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

normalizer = n_agents*reward.expec; % n_agents*(n_agents - 1)/2;
r = r/normalizer;
if nargout >= 2
	drdx = drdx/normalizer;
	if fast
		g = 0;
	else
		g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
	end
end
if nargout >= 6
	d2rdxdx = d2rdxdx/normalizer;
    %disp(reward.type)
end