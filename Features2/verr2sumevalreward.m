function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	verr2sumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

fast = isfield(reward, 'fast') && reward.fast;

normalizer = mdp_data.n_ped*reward.expec;

Nt = size(states, 1);
Nu = mdp_data.udims;
Nx = mdp_data.dims;

if reward.type_other
    idx = mdp_data.idx_other;
elseif reward.type_w
    idx = mdp_data.idx_wheelchair;
elseif reward.type_c
    idx = mdp_data.idx_companion;
else
    idx = 1:(Nu/2);
end

n = 2*length(idx);
jj = Nu + reshape([idx*2 - 1; idx*2], [1, n]);

%idx = (1:Nu) + Nu;
err = states(:, jj) - mdp_data.v_des(jj - Nu);
r = 0.5*sum(err.^2, 2)/normalizer;
if nargout >= 2
	drdx = zeros(Nt, Nx);
	drdx(:, jj) = err/normalizer;
	if fast
		g = 0;
	else
		g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
	end
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
	d2rdxdx(:, jj, jj) = repmat(reshape(eye(n), [1, n, n]), [Nt, 1, 1])/normalizer;
    %disp(reward.type)
end