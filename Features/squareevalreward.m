function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	squareevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

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

r = 0.5*hr.^2;

if nargout >= 2
	% Compute gradient.
	drdx = hr.*hdrdx;
	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end

if nargout >= 3
	% Gradients with respect to controls are always zero.
	drdu = zeros(T, Du);
	d2rdudu = zeros(T, Du, Du);
end

if nargout >= 6
	% drdx has already been computed.
	d2rdxdx = hdrdx.*reshape(hdrdx, [T, 1, Dx]) + hr.*hd2rdxdx;
end