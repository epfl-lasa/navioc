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

V2 = Vx.^2 + Vy.^2;
Vmag = sqrt(V2);
VcrossA = Vx.*Ay - Vy.*Ax;
Aleft = VcrossA./Vmag;

r = 0.5*sum(Aleft.^2, 2);

if nargout >= 2
	dAleftdVx = (Ay.*Vmag - Aleft.*Vx)./V2;
	dAleftdVy = (-Ax.*Vmag - Aleft.*Vy)./V2;
	dAleftdAx = -Vy./Vmag;
	dAleftdAy = Vx./Vmag;

	drdx = zeros(Nt, Nx);
	drdx(:, vx_jj) = Aleft.*dAleftdVx;
	drdx(:, vy_jj) = Aleft.*dAleftdVy;
	drdx(:, ax_jj) = Aleft.*dAleftdAx;
	drdx(:, ay_jj) = Aleft.*dAleftdAy;

	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end
if nargout >= 3
	drdu = zeros(Nt, Nu);
	d2rdudu = zeros(Nt, Nu, Nu);
end
if nargout >= 6
	d2AleftdVxdVx = ((Ay.*Vx./Vmag - Aleft - Vx.*dAleftdVx).*V2 - (...
		Ay.*Vmag - Aleft.*Vx).*Vx*2)./V2.^2;
	d2AleftdVxdVy = ((Ay.*Vy./Vmag - Vx.*dAleftdVy).*V2 - (...
		Ay.*Vmag - Aleft.*Vx).*Vy*2)./V2.^2;
	d2AleftdVxdAx = Vx.*Vy./V2./Vmag;
	d2AleftdVxdAy = (1 - Vx.^2./V2)./Vmag;

	d2AleftdVydVy = ((-Ax.*Vy./Vmag - Aleft - Vy.*dAleftdVy).*V2 - (...
		-Ax.*Vmag - Aleft.*Vy).*Vy*2)./V2.^2;
	d2AleftdVydAx = (-1 + Vy.^2./V2)./Vmag;
	d2AleftdVydAy = -Vx.*Vy./V2./Vmag;

	d2rdxdx = zeros(Nt, Nx, Nx);

	base = repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	
	d2rdxdx(:, vx_jj, vx_jj) = (Aleft.*d2AleftdVxdVx + dAleftdVx.^2).*base;
	d2rdxdx(:, vx_jj, vy_jj) = (Aleft.*d2AleftdVxdVy + dAleftdVx.*dAleftdVy).*base;
	d2rdxdx(:, vx_jj, ax_jj) = (Aleft.*d2AleftdVxdAx + dAleftdVx.*dAleftdAx).*base;
	d2rdxdx(:, vx_jj, ay_jj) = (Aleft.*d2AleftdVxdAy + dAleftdVx.*dAleftdAy).*base;

	d2rdxdx(:, vy_jj, vx_jj) = d2rdxdx(:, vx_jj, vy_jj);
	d2rdxdx(:, vy_jj, vy_jj) = (Aleft.*d2AleftdVydVy + dAleftdVy.^2).*base;
	d2rdxdx(:, vy_jj, ax_jj) = (Aleft.*d2AleftdVydAx + dAleftdVy.*dAleftdAx).*base;
	d2rdxdx(:, vy_jj, ay_jj) = (Aleft.*d2AleftdVydAy + dAleftdVy.*dAleftdAy).*base;

	d2rdxdx(:, ax_jj, vx_jj) = d2rdxdx(:, vx_jj, ax_jj);
	d2rdxdx(:, ax_jj, vy_jj) = d2rdxdx(:, vy_jj, ax_jj);
	d2rdxdx(:, ax_jj, ax_jj) = dAleftdAx.^2.*base;
	d2rdxdx(:, ax_jj, ay_jj) = dAleftdAx.*dAleftdAy.*base;

	d2rdxdx(:, ay_jj, vx_jj) = d2rdxdx(:, vx_jj, ay_jj);
	d2rdxdx(:, ay_jj, vy_jj) = d2rdxdx(:, vy_jj, ay_jj);
	d2rdxdx(:, ay_jj, ax_jj) = d2rdxdx(:, ax_jj, ay_jj);
	d2rdxdx(:, ay_jj, ay_jj) = dAleftdAy.^2.*base;
end

r = -r;
if nargout >= 2
	drdx = -drdx;
	g = -g;
end
if nargout >= 6
	d2rdxdx = -d2rdxdx;
end