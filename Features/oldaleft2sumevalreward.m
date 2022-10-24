function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	aleft2sumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

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

VcrossA = Vx.*Ay - Vy.*Ax;
r = 0.5*sum(VcrossA.^2, 2);

if nargout >= 2
	drdx = zeros(Nt, Nx);
	drdx(:, vx_jj) = Ay.*VcrossA;
	drdx(:, vy_jj) = -Ax.*VcrossA;
	drdx(:, ax_jj) = -Vy.*VcrossA;
	drdx(:, ay_jj) = Vx.*VcrossA;
	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end
if nargout >= 3
	drdu = zeros(Nt, Nu);
	d2rdudu = zeros(Nt, Nu, Nu);
end
if nargout >= 6
	d2rdxdx = zeros(Nt, Nx, Nx);

	d2rdxdx(:, vx_jj, vx_jj) = Ay.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	d2rdxdx(:, vx_jj, vy_jj) = -Ay.*Ax.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	d2rdxdx(:, vx_jj, ax_jj) = -Ay.*Vy.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	d2rdxdx(:, vx_jj, ay_jj) = (2*Vx.*Ay - Vy.*Ax).*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);

	d2rdxdx(:, vy_jj, vx_jj) = permute(d2rdxdx(:, vx_jj, vy_jj), [1, 3, 2]);
	d2rdxdx(:, vy_jj, vy_jj) = Ax.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	d2rdxdx(:, vy_jj, ax_jj) = (2*Vy.*Ax - Vx.*Ay).*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	d2rdxdx(:, vy_jj, ay_jj) = -Ax.*Vx.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);

	d2rdxdx(:, ax_jj, vx_jj) = permute(d2rdxdx(:, vx_jj, ax_jj), [1, 3, 2]);
	d2rdxdx(:, ax_jj, vy_jj) = permute(d2rdxdx(:, vy_jj, ax_jj), [1, 3, 2]);
	d2rdxdx(:, ax_jj, ax_jj) = Vy.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	d2rdxdx(:, ax_jj, ay_jj) = -Vx.*Vy.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);

	d2rdxdx(:, ay_jj, vx_jj) = permute(d2rdxdx(:, vx_jj, ay_jj), [1, 3, 2]);
	d2rdxdx(:, ay_jj, vy_jj) = permute(d2rdxdx(:, vy_jj, ay_jj), [1, 3, 2]);
	d2rdxdx(:, ay_jj, ax_jj) = permute(d2rdxdx(:, ax_jj, ay_jj), [1, 3, 2]);
	d2rdxdx(:, ay_jj, ay_jj) = Vx.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
    %disp(reward.type)
end

r = -r;
if nargout >= 2
	drdx = -drdx;
	g = -g;
end
if nargout >= 6
	d2rdxdx = -d2rdxdx;
end