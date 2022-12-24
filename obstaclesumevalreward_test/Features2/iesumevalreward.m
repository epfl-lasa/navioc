function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = iesumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

fast = isfield(reward, 'fast') && reward.fast;

% parameters
R = reward.R;
R2 = R^2;
eps1 = 0.22*R2;
eps2 = reward.eps2; %0.01;
s = reward.a;

% dimensions
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
		iv1 = ip1 + Nu;
		iv2 = ip2 + Nu;

		P = states(:, ip1) - states(:, ip2);
		V = states(:, iv1) - states(:, iv2);

		P2 = sum(P.^2, 2);
		V2 = sum(V.^2, 2);
		Eta = V2 + eps2;

		Pmag = sqrt(P2);
		Vmag = sqrt(V2);

		% Hack!
		Vmag(Vmag < 1e-10) = 1e-10; % avoids division by zero later on

		Mu = -sum(P.*V, 2);
		Rho = Pmag/R - 1.0;
		Delta = sqrt(P2 + R2);
		Mu_Eta = Mu./Eta;

		MPD2 = P2 - Mu.*Mu_Eta;
		D = eps1 + Rho.*(Rho*R2 + 2*MPD2);

		Z = s*(Mu - P2.*Vmag./Delta);
		H = 1./(1 + exp(-Z));

		r = r + V2.*H./D;

		if nargout >= 2 
			% dMPD2dx = [dMPD2dP1x, dMPD2dP1y, dMPD2dP2x, dMPD2dP2y, dMPD2dV1x, dMPD2dV1y, dMPD2dV2x, dMPD2dV2y]
			% dMPD2dx = [dMPD2dP1, dMPD2dP2, dMPD2dV1, dMPD2dV2]
			% dMPD2dx = [dMPD2dP1, -dMPD2dP1, dMPD2dV1, -dMPD2dV1]

			dMudx = [-V, V, -P, P]; 

			dMPD2dP12 = 2*[P, -P];
			dMPD2dV12 = 2*Mu_Eta.^2.*[V, -V];
			dMPD2dx = [dMPD2dP12, dMPD2dV12] - 2*Mu_Eta.*dMudx;

			dRhodP = [P, -P]./Pmag/R;
			dDdx = [2*(Rho*R2 + MPD2).*dRhodP, zeros(size(P, 1), 4)] + 2*Rho.*dMPD2dx;

			dDeltadP = [P, -P]./Delta;
			dZdx = s*(dMudx - [2*Vmag.*[P, -P] - P2.*Vmag.*dDeltadP./Delta, P2./Vmag.*[V, -V]]./Delta);
			dHdx = H.*(1 - H).*dZdx;

            jj = [ip1, ip2, iv1, iv2];
            
			drdx(:, jj) = drdx(:, jj) + H./D.^2.*(2*D.*[zeros(size(P, 1), 4), V, -V] - V2.*dDdx) + V2./D.*dHdx;
		end
		if nargout >= 6
			% Hessian
			m = size(V, 1); % = Nt

			dEtadx = [zeros(m, 4), 2*[V, -V]];
			dRhodx = [dRhodP, zeros(m, 4)];
			dDeltadx = [dDeltadP, zeros(m, 4)];

			I_ = [eye(2), -eye(2); -eye(2), eye(2)];
			dPdxTdPdx = repmat(reshape([I_, zeros(4); zeros(4, 8)], [1, 8, 8]), [m, 1, 1]);
			dVdxTdVdx = repmat(reshape([zeros(4, 8); zeros(4), I_], [1, 8, 8]), [m, 1, 1]);
			dPdxTdVdx = repmat(reshape([zeros(4), I_; zeros(4, 8)], [1, 8, 8]), [m, 1, 1]);
			dVdxTdPdx = repmat(reshape([zeros(4, 8); I_, zeros(4)], [1, 8, 8]), [m, 1, 1]);

			dPdxTP = [P, -P, zeros(m, 4)];
			dVdxTV = [zeros(m, 4), V, -V];

			d2Mudx2 = -dPdxTdVdx - dVdxTdPdx;
			d2Rhodx2 = (Pmag.*dPdxTdPdx - dPdxTP.*reshape(dPdxTP, [m, 1, 8])./Pmag)./P2/R;
			d2MPD2dx2 = 2*dPdxTdPdx - 1./Eta.^4.*(...
				Eta.^2.*(2*(Mu.*Eta.*d2Mudx2 + dMudx.*reshape((Mu.*dEtadx + Eta.*dMudx), [m, 1, 8])) ...
					- 2*(2*dVdxTV.*Mu.*reshape(dMudx, [m, 1, 8]) + Mu.^2.*dVdxTdVdx)) ...
				- (2*Mu.*Eta.*dMudx - 2*Mu.^2.*dVdxTV).*2.*Eta.*reshape(dEtadx, [m, 1, 8]) ...
			);
			d2Ddx2 = dRhodx.*reshape(R2*dRhodx + 2*dMPD2dx, [m, 1, 8]) + (Rho*R2 + 2*MPD2).*d2Rhodx2 + ...
				(R2*dRhodx + 2*dMPD2dx).*reshape(dRhodx, [m, 1, 8]) + Rho.*(R2*d2Rhodx2 + 2*d2MPD2dx2);

			d2Deltadx2 = 1./Delta.^2.*(Delta.*dPdxTdPdx - dPdxTP.*reshape(dDeltadx, [m, 1, 8]));
			dBdx = 1./Delta.^4.*(Delta.^2.*(dDeltadx.*reshape(2*Vmag.*dPdxTP + P2./Vmag.*dVdxTV, [m, 1, 8]) + P2.*Vmag.*d2Deltadx2) ...
				- 2*Delta.*P2.*Vmag.*dDeltadx.*reshape(dDeltadx, [m, 1, 8]));
			dAdx = 1./Delta.^2.*(Delta.*(2*(Vmag.*dPdxTdPdx + dPdxTP.*reshape(dVdxTV, [m, 1, 8])./Vmag) ...
					+ 2*dVdxTV.*reshape(dPdxTP, [m, 1, 8])./Vmag + P2./Vmag.*(dVdxTdVdx - dVdxTV.*reshape(dVdxTV, [m, 1, 8])./V2)) ...
				- (2*Vmag.*dPdxTP + P2./Vmag.*dVdxTV).*reshape(dDeltadx, [m, 1, 8]) ...
			);
			d2Zdx2 = s*(d2Mudx2 - dAdx + dBdx);
			d2Hdx2 = dZdx.*reshape(dHdx, [m, 1, 8]).*(1 - 2*H) + H.*(1 - H).*d2Zdx2;

			d2rdxdx(:, jj, jj) = d2rdxdx(:, jj, jj) + (2*D.*dVdxTV - V2.*dDdx).*reshape(dHdx./D.^2 - 2*H./D.^3.*dDdx, [m, 1, 8]) ...
				+ H./D.^2.*(2*dVdxTV.*reshape(dDdx, [m, 1, 8]) + 2*D.*dVdxTdVdx - 2*dDdx.*reshape(dVdxTV, [m, 1, 8]) ...
				- V2.*d2Ddx2) + dHdx.*reshape((2*D.*dVdxTV - V2.*dDdx)./D.^2, [m, 1, 8]) + V2./D.*d2Hdx2;
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

% function checkvalues(X)
% if sum(any(isnan(X)))
% 	disp('NaN')
% 	disp(X(isnan(X)))
% end
% if sum(any(isinf(X)))
% 	disp('Inf')
% 	disp(X(isinf(X)))
% end