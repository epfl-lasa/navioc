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
			%d2rdp2dp2 = d2rdp1dp1;
			%d2rdp1dp2 = k.*(eye(Nt, 2, 2) - 4*P2inv.*P.*reshape(P, [Nt, 1, 2]));
			d2rdxdx(:, ip1, ip1) = d2rdxdx(:, ip1, ip1) + d2rdp1dp1;
			d2rdxdx(:, ip1, ip2) = d2rdxdx(:, ip1, ip2) - d2rdp1dp1;
			d2rdxdx(:, ip2, ip1) = d2rdxdx(:, ip2, ip1) - d2rdp1dp1;
			d2rdxdx(:, ip2, ip2) = d2rdxdx(:, ip2, ip2) + d2rdp1dp1;
		end
	end
end

if nargout >= 2
	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end


% %% utilities (could be included in mdp_data)
% I1 = zeros(1, Np);
% I2 = zeros(1, Np);
% idxpairmat = zeros(Nag - 1, Nag);
% k = 0;
% for i = 1:Nag
% 	for j = (i + 1):Nag
% 		k = k + 1;
% 		I1(k) = i;
% 		I2(k) = j;
% 		idxpairmat(j - 1, i) = k;
% 		idxpairmat(i, j) = k;
% 	end
% end
% idxpair = reshape(idxpairmat, [Np*2, 1]);

% px1_jj = I1*2 - 1;
% py1_jj = I1*2;
% px2_jj = I2*2 - 1;
% py2_jj = I2*2;

% px_jj = 2*(1:Nag) - 1;
% py_jj = 2*(1:Nag);

% %%

% Dx = states(:, px1_jj) - states(:, px2_jj);
% Dy = states(:, py1_jj) - states(:, py2_jj);

% dinv2 = 1./(Dx.^2 + Dy.^2);

% r = sum(dinv2, 2);

% if nargout >= 2
% 	drdx = zeros(Nt, Nx);

% 	k = -2*dinv2.^2;

% 	drpairdpx1 = k.*Dx;
% 	drpairdpy1 = k.*Dy;
% 	drpairdpx2 = -drpairdpx1;
% 	drpairdpx2 = -drpairdpy1;

% 	drdx() = permute(sum(reshape(drpairdpx1(:, idxpair), [Nt, Nag - 1, Nag]), 2), [1, 3, 2]);

% 	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
% end


% idx = (1:Nu) + Nu*reward.order;
% err = states(:, idx) - mdp_data.x_des{reward.order};% reward.x_des;
% r = 0.5*sum(err.^2, 2);
% if nargout >= 2
% 	drdx = zeros(Nt, Nx);
% 	drdx(:, idx) = err;
% 	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
% end
% if nargout >= 3
% 	drdu = zeros(Nt, Nu);
% 	d2rdudu = zeros(Nt, Nu, Nu);
% end
% if nargout >= 6
% 	d2rdxdx = zeros(Nt, Nx, Nx);
% 	d2rdxdx(:, idx, idx) = repmat(reshape(eye(Nu), [1, Nu, Nu]), [Nt, 1, 1]);
%     %disp(reward.type)
% end