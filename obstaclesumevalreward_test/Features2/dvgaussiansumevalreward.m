function [r, g_, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	dvgaussiansumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

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
		iv1 = ip1 + Nu;
		iv2 = ip2 + Nu;

		P = states(:, ip1) - states(:, ip2);
		P2 = sum(P.^2, 2);
		V = states(:, iv1) - states(:, iv2); 
		V2 = sum(V.^2, 2);

		g = exp(-0.5*P2/reward.sigma_p^2);
		h = exp(-0.5*V2/reward.sigma_v^2);

		r = r + g.*h;

		if nargout >= 2
			k = -g/reward.sigma_p^2;
			l = -h/reward.sigma_v^2;

			dgdp1 = k.*P;
			dhdv1 = l.*V;

			dfdp1 = dgdp1.*h;
			dfdv1 = dhdv1.*g;

			drdx(:, ip1) = drdx(:, ip1) + dfdp1;
			drdx(:, ip2) = drdx(:, ip2) - dfdp1;
			drdx(:, iv1) = drdx(:, iv1) + dfdv1;
			drdx(:, iv2) = drdx(:, iv2) - dfdv1;
		end

		if nargout >= 6
			d2gdp1dp1 = k.*(reshape(eye(2), [1, 2, 2]) - (1/reward.sigma_p^2)*P.*reshape(P, [Nt, 1, 2]));
			d2hdv1dv1 = l.*(reshape(eye(2), [1, 2, 2]) - (1/reward.sigma_v^2)*V.*reshape(V, [Nt, 1, 2]));

			d2fdp1dp1 = d2gdp1dp1.*h;
			d2fdv1dv1 = d2hdv1dv1.*g;

			d2rdxdx(:, ip1, ip1) = d2rdxdx(:, ip1, ip1) + d2fdp1dp1;
			d2rdxdx(:, ip1, ip2) = d2rdxdx(:, ip1, ip2) - d2fdp1dp1;
			d2rdxdx(:, ip2, ip1) = d2rdxdx(:, ip2, ip1) - d2fdp1dp1;
			d2rdxdx(:, ip2, ip2) = d2rdxdx(:, ip2, ip2) + d2fdp1dp1;
			d2rdxdx(:, iv1, iv1) = d2rdxdx(:, iv1, iv1) + d2fdv1dv1;
			d2rdxdx(:, iv1, iv2) = d2rdxdx(:, iv1, iv2) - d2fdv1dv1;
			d2rdxdx(:, iv2, iv1) = d2rdxdx(:, iv2, iv1) - d2fdv1dv1;
			d2rdxdx(:, iv2, iv2) = d2rdxdx(:, iv2, iv2) + d2fdv1dv1;

			d2fdp1dv1 = dgdp1.*reshape(dhdv1, [Nt, 1, 2]);
			d2fdv1dp1 = dhdv1.*reshape(dgdp1, [Nt, 1, 2]);
			d2rdxdx(:, ip1, iv1) = d2rdxdx(:, ip1, iv1) + d2fdp1dv1;
			d2rdxdx(:, iv1, ip1) = d2rdxdx(:, iv1, ip1) + d2fdv1dp1;
			d2rdxdx(:, ip2, iv1) = d2rdxdx(:, ip2, iv1) - d2fdp1dv1;
			d2rdxdx(:, iv1, ip2) = d2rdxdx(:, iv1, ip2) - d2fdv1dp1;
			d2rdxdx(:, ip1, iv2) = d2rdxdx(:, ip1, iv2) - d2fdp1dv1;
			d2rdxdx(:, iv2, ip1) = d2rdxdx(:, iv2, ip1) - d2fdv1dp1;
			d2rdxdx(:, ip2, iv2) = d2rdxdx(:, ip2, iv2) + d2fdp1dv1;
			d2rdxdx(:, iv2, ip2) = d2rdxdx(:, iv2, ip2) + d2fdv1dp1;
		end
	end
end

normalizer = n_agents*reward.expec; % n_agents*(n_agents - 1)/2;
r = r/normalizer;
if nargout >= 2
	drdx = drdx/normalizer;
	g_ = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end
if nargout >= 6
	d2rdxdx = d2rdxdx/normalizer;
    %disp(reward.type)
end