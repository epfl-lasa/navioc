function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	omega2sumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

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

V2 = Vx.^2 + Vy.^2;
%Vmag = sqrt(V2);
VcrossA = Vx.*Ay - Vy.*Ax;
Omega = VcrossA./V2;

r = 0.5*sum(Omega.^2, 2);

if nargout >= 2
	dOmdVx = (Ay.*V2 - 2*VcrossA.*Vx)./V2.^2;
	dOmdVy = (-Ax.*V2 - 2*VcrossA.*Vy)./V2.^2;
	dOmdAx = -Vy./V2;
	dOmdAy = Vx./V2;

	drdx = zeros(Nt, Nx);
	drdx(:, vx_jj) = Omega.*dOmdVx;
	drdx(:, vy_jj) = Omega.*dOmdVy;
	drdx(:, ax_jj) = Omega.*dOmdAx;
	drdx(:, ay_jj) = Omega.*dOmdAy;

	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end

if nargout >= 3
	drdu = zeros(Nt, Nu);
	d2rdudu = zeros(Nt, Nu, Nu);
end
if nargout >= 6
	d2OmdVxdVx = (-6*Vx.*Vy.*(Vx.*Ax + Vy.*Ay) + 2*(Vx.^3.*Ay + Vy.^3.*Ax))./V2.^3;
	d2OmdVxdVy = (6*Vx.*Vy.*VcrossA + 2*(Vx.^3.*Ax - Vy.^3.*Ay))./V2.^3;
	d2OmdVxdAx = 2*Vx.*Vy./V2.^2;
	d2OmdVxdAy = (-2*Vx.^2 + V2)./V2.^2;

	d2OmdVydVy = -d2OmdVxdVx;
	d2OmdVydAx = (2*Vy.^2 - V2)./V2.^2;
	d2OmdVydAy = -d2OmdVxdAx;

	d2rdxdx = zeros(Nt, Nx, Nx);

	base = repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	
	d2rdxdx(:, vx_jj, vx_jj) = (Omega.*d2OmdVxdVx + dOmdVx.^2).*base;
	d2rdxdx(:, vx_jj, vy_jj) = (Omega.*d2OmdVxdVy + dOmdVx.*dOmdVy).*base;
	d2rdxdx(:, vx_jj, ax_jj) = (Omega.*d2OmdVxdAx + dOmdVx.*dOmdAx).*base;
	d2rdxdx(:, vx_jj, ay_jj) = (Omega.*d2OmdVxdAy + dOmdVx.*dOmdAy).*base;

	d2rdxdx(:, vy_jj, vx_jj) = d2rdxdx(:, vx_jj, vy_jj);
	d2rdxdx(:, vy_jj, vy_jj) = (Omega.*d2OmdVydVy + dOmdVy.^2).*base;
	d2rdxdx(:, vy_jj, ax_jj) = (Omega.*d2OmdVydAx + dOmdVy.*dOmdAx).*base;
	d2rdxdx(:, vy_jj, ay_jj) = (Omega.*d2OmdVydAy + dOmdVy.*dOmdAy).*base;

	d2rdxdx(:, ax_jj, vx_jj) = d2rdxdx(:, vx_jj, ax_jj);
	d2rdxdx(:, ax_jj, vy_jj) = d2rdxdx(:, vy_jj, ax_jj);
	d2rdxdx(:, ax_jj, ax_jj) = dOmdAx.^2.*base;
	d2rdxdx(:, ax_jj, ay_jj) = dOmdAx.*dOmdAy.*base;

	d2rdxdx(:, ay_jj, vx_jj) = d2rdxdx(:, vx_jj, ay_jj);
	d2rdxdx(:, ay_jj, vy_jj) = d2rdxdx(:, vy_jj, ay_jj);
	d2rdxdx(:, ay_jj, ax_jj) = d2rdxdx(:, ax_jj, ay_jj);
	d2rdxdx(:, ay_jj, ay_jj) = dOmdAy.^2.*base;
end