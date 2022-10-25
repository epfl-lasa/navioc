function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	gaussianevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

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

d = (hr - reward.mu)/reward.sigma;

r = exp(-0.5*d.^2);

if nargout >= 2
	% Compute gradient.
	k = -d/reward.sigma;

	dfdh = k.*r;

	drdx = dfdh.*hdrdx;

	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end

if nargout >= 3
	% Gradients with respect to controls are always zero.
	drdu = zeros(T, Du);
	d2rdudu = zeros(T, Du, Du);
end

if nargout >= 6
	% drdx has already been computed.
	d2rdxdx = (k.*dfdh - r/reward.sigma^2).*hdrdx.*reshape(hdrdx, [T, 1, Dx]) + dfdh.*hd2rdxdx;

	%dfdh.*hd2rdxdx + r.*(k.^2 - 1/reward.sigma^2).*hdrdx.*reshape(hdrdx, [T, 1, Dx]);

	%dfdh.*hd2rdxdx + (k.*dfdh - r/reward.sigma^2).*hdrdx.*reshape(hdrdx, [T, 1, Dx]);
end