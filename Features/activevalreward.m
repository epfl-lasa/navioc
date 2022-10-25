function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	activevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

% Get constants.
T = size(u, 1);
Du = size(u, 2);
Dx = size(states, 2);

f = strcat(reward.feature.type, 'evalreward');
if nargout >= 6
	[hr, hg, hdrdu, hd2rdudu, hdrdx, hd2rdxdx] = feval(f, ...
	        reward.feature, mdp_data, x, u, states, A, B);
elseif nargout >= 2
	[hr, ~, ~, ~, hdrdx] = feval(f, ...
	        reward.feature, mdp_data, x, u, states, A, B);
else
	hr = feval(f, ...
	        reward.feature, mdp_data, x, u, states);
end

r = 1./(1 + exp(-reward.a*(hr - reward.c)));

if nargout >= 2
	% Compute gradient.
	pre_factor = reward.a*r.^2.*exp(-reward.a*(hr - reward.c));
	drdx = pre_factor.*hdrdx;
	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end

if nargout >= 3
	% Gradients with respect to controls are always zero.
	drdu = zeros(T, Du);
	d2rdudu = zeros(T, Du, Du);
end

if nargout >= 6
	% drdx has already been computed.
	d2rdxdx = pre_factor.*hd2rdxdx + ...
		reward.a*(2*r.*exp(-reward.a*(hr - reward.c)) - 1).*hdrdx.*reshape(hdrdx, [T, 1, Dx]);
end