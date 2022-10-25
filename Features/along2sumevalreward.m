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

V2 = Vx.^2 + Vy.^2;
Vmag = sqrt(V2);
Along = (Vx.*Ax + Vy.*Ay)./Vmag;

r = 0.5*sum(Along.^2, 2);

if nargout >= 2
	dAlongdVx = (Ax.*Vmag - Along.*Vx)./V2;
	dAlongdVy = (Ay.*Vmag - Along.*Vy)./V2;
	dAlongdAx = Vx./Vmag;
	dAlongdAy = Vy./Vmag;

	drdx = zeros(Nt, Nx);
	drdx(:, vx_jj) = Along.*dAlongdVx;
	drdx(:, vy_jj) = Along.*dAlongdVy;
	drdx(:, ax_jj) = Along.*dAlongdAx;
	drdx(:, ay_jj) = Along.*dAlongdAy;

	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end
if nargout >= 3
	drdu = zeros(Nt, Nu);
	d2rdudu = zeros(Nt, Nu, Nu);
end
if nargout >= 6
	d2AlongdVxdVx = ((Ax.*Vx./Vmag - Along - Vx.*dAlongdVx).*V2 - (...
		Ax.*Vmag - Along.*Vx).*Vx*2)./V2.^2;

	d2AlongdVxdVy = ((Ax.*Vy./Vmag - Vx.*dAlongdVy).*V2 - (...
		Ax.*Vmag - Along.*Vx).*Vy*2)./V2.^2;

	d2AlongdVxdAx = (1 - Vx.^2./V2)./Vmag;

	d2AlongdVxdAy = -Vy./V2.*Vx./Vmag;

	d2AlongdVydVy = ((Ay.*Vy./Vmag - Along - Vy.*dAlongdVy).*V2 - (...
		Ay.*Vmag - Along.*Vy).*Vy*2)./V2.^2;

	d2AlongdVydAx = -Vx./V2.*Vy./Vmag;

	d2AlongdVydAy = (1 - Vy.^2./V2)./Vmag;

	d2rdxdx = zeros(Nt, Nx, Nx);

	base = repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]);
	
	d2rdxdx(:, vx_jj, vx_jj) = (Along.*d2AlongdVxdVx + dAlongdVx.^2).*base;
	d2rdxdx(:, vx_jj, vy_jj) = (Along.*d2AlongdVxdVy + dAlongdVx.*dAlongdVy).*base;
	d2rdxdx(:, vx_jj, ax_jj) = (Along.*d2AlongdVxdAx + dAlongdVx.*dAlongdAx).*base;
	d2rdxdx(:, vx_jj, ay_jj) = (Along.*d2AlongdVxdAy + dAlongdVx.*dAlongdAy).*base;

	d2rdxdx(:, vy_jj, vx_jj) = d2rdxdx(:, vx_jj, vy_jj);
	d2rdxdx(:, vy_jj, vy_jj) = (Along.*d2AlongdVydVy + dAlongdVy.^2).*base;
	d2rdxdx(:, vy_jj, ax_jj) = (Along.*d2AlongdVydAx + dAlongdVy.*dAlongdAx).*base;
	d2rdxdx(:, vy_jj, ay_jj) = (Along.*d2AlongdVydAy + dAlongdVy.*dAlongdAy).*base;

	d2rdxdx(:, ax_jj, vx_jj) = d2rdxdx(:, vx_jj, ax_jj);
	d2rdxdx(:, ax_jj, vy_jj) = d2rdxdx(:, vy_jj, ax_jj);
	d2rdxdx(:, ax_jj, ax_jj) = dAlongdAx.^2.*base;
	d2rdxdx(:, ax_jj, ay_jj) = dAlongdAx.*dAlongdAy.*base;

	d2rdxdx(:, ay_jj, vx_jj) = d2rdxdx(:, vx_jj, ay_jj);
	d2rdxdx(:, ay_jj, vy_jj) = d2rdxdx(:, vy_jj, ay_jj);
	d2rdxdx(:, ay_jj, ax_jj) = d2rdxdx(:, ax_jj, ay_jj);
	d2rdxdx(:, ay_jj, ay_jj) = dAlongdAy.^2.*base;
end