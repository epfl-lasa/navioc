function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	accsmabssumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

normalizer = mdp_data.n_ped*reward.expec;

Nt = size(u, 1);
Nu = size(u, 2);
Nx = size(states, 2);

if reward.type_other
    idx = mdp_data.idx_other;
elseif reward.type_w
    idx = mdp_data.idx_wheelchair;
elseif reward.type_c
    idx = mdp_data.idx_companion;
else
    idx = 1:(Nu/2);
end

ax_jj = idx*2 - 1;
ay_jj = ax_jj + 1;

Ax = states(:, ax_jj);
Ay = states(:, ay_jj);

A2 = Ax.^2 + Ay.^2;
Z = sqrt(A2);

s = reward.s;

Expz = exp(-2*s*Z);
Fz = Z + 1/s*log(1 + Expz);

r = sum(Fz, 2)/normalizer;
if nargout >= 2
    g = zeros(Nt, Nu);
    dFdzOverZ = (1 - 2*Expz./(1 + Expz))/normalizer./Z;
	g(:, ax_jj) = dFdzOverZ.*Ax;
	g(:, ay_jj) = dFdzOverZ.*Ay;
end

if nargout >= 3
    drdu = g;
    d2rdudu = zeros(Nt, Nu, Nu);

    d2FdzdzOverA2 = 4*s*Expz./(1 + Expz).^2/normalizer./A2;
	n = length(idx);
	mask = reshape(eye(n), [1, n, n]);
	d2rdudu(:, ax_jj, ax_jj) = (d2FdzdzOverA2.*Ax.^2 + ...
		dFdzOverZ.*(1 - Ax.^2./A2)).*mask;

	d2rdudu(:, ax_jj, ay_jj) = (d2FdzdzOverA2.*Ax.*Ay + ...
		dFdzOverZ.*(0 - Ax.*Ay./A2)).*mask;

	d2rdudu(:, ay_jj, ax_jj) = d2rdudu(:, ax_jj, ay_jj);

	d2rdudu(:, ay_jj, ay_jj) = (d2FdzdzOverA2.*Ay.^2 + ...
		dFdzOverZ.*(1 - Ay.^2./A2)).*mask;
end

if nargout >= 5
    drdx = zeros(Nt, Nx);
    d2rdxdx = zeros(Nt, Nx, Nx);
end