function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	along2sumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

Nt = size(u, 1);
Nu = size(u, 2);
Nx = size(states, 2);
Nag = Nu/2;

vx_jj = (1:2:Nu) + Nu;
vy_jj = vx_jj + 1;
ax_jj = vx_jj + Nu;
ay_jj = ax_jj + 1;

Vx = states(:, vx_jj);
Vy = states(:, vy_jj);
Ax = states(:, ax_jj);
Ay = states(:, ay_jj);

VdotA = Vx.*Ax + Vy.*Ay;
r = 0.5*sum(VdotA.^2, 2);

if nargout >= 2
	drdx = zeros(Nt, Nx);
	drdx(:, vx_jj) = VdotA.*Ax;
	drdx(:, vy_jj) = VdotA.*Ay;
	drdx(:, ax_jj) = VdotA.*Vx;
	drdx(:, ay_jj) = VdotA.*Vy;
	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end
if nargout >= 3
	drdu = zeros(Nt, Nu);
	d2rdudu = zeros(Nt, Nu, Nu);
end
if nargout >= 6
	d2rdxdx = zeros(Nt, Nx, Nx);

	d2rdxdx(:, vx_jj, vx_jj) = Ax.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	d2rdxdx(:, vx_jj, vy_jj) = Ax.*Ay.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	d2rdxdx(:, vx_jj, ax_jj) = (2*Vx.*Ax + Vy.*Ay).*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	d2rdxdx(:, vx_jj, ay_jj) = Vy.*Ax.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);

	d2rdxdx(:, vy_jj, vx_jj) = d2rdxdx(:, vx_jj, vy_jj);
	d2rdxdx(:, vy_jj, vy_jj) = Ay.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	d2rdxdx(:, vy_jj, ax_jj) = Vx.*Ay.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	d2rdxdx(:, vy_jj, ay_jj) = (Vx.*Ax + 2*Vy.*Ay).*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);

	d2rdxdx(:, ax_jj, vx_jj) = d2rdxdx(:, vx_jj, ax_jj);
	d2rdxdx(:, ax_jj, vy_jj) = d2rdxdx(:, vy_jj, ax_jj);
	d2rdxdx(:, ax_jj, ax_jj) = Vx.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	d2rdxdx(:, ax_jj, ay_jj) = Vx.*Vy.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);

	d2rdxdx(:, ay_jj, vx_jj) = d2rdxdx(:, vx_jj, ay_jj);
	d2rdxdx(:, ay_jj, vy_jj) = d2rdxdx(:, vy_jj, ay_jj);
	d2rdxdx(:, ay_jj, ax_jj) = d2rdxdx(:, ax_jj, ay_jj);
	d2rdxdx(:, ay_jj, ay_jj) = Vy.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
    %disp(reward.type)
end