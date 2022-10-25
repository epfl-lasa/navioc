function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	along4sumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

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
r1 = 0.5*sum(VdotA.^2, 2);
r = 0.5*r1.^2;

if nargout >= 2
	drdx = zeros(Nt, Nx);
	AxVdotA = Ax.*VdotA;
	AyVdotA = Ay.*VdotA;
	VxVdotA = Vx.*VdotA;
	VyVdotA = Vy.*VdotA;
	drdx(:, vx_jj) = AxVdotA.*r1;
	drdx(:, vy_jj) = AyVdotA.*r1;
	drdx(:, ax_jj) = VxVdotA.*r1;
	drdx(:, ay_jj) = VyVdotA.*r1;
	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end
if nargout >= 3
	drdu = zeros(Nt, Nu);
	d2rdudu = zeros(Nt, Nu, Nu);
end
if nargout >= 6
	d2rdxdx = zeros(Nt, Nx, Nx);

	d2rdxdx(:, vx_jj, vx_jj) = AxVdotA.*reshape(AxVdotA, [Nt, 1, Nag]) + ...
		Ax.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;
	d2rdxdx(:, vx_jj, vy_jj) = AxVdotA.*reshape(AyVdotA, [Nt, 1, Nag]) + ...
		Ax.*Ay.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;
	d2rdxdx(:, vx_jj, ax_jj) = AxVdotA.*reshape(VxVdotA, [Nt, 1, Nag]) + ...
		(2*Vx.*Ax + Vy.*Ay).*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;
	d2rdxdx(:, vx_jj, ay_jj) = AxVdotA.*reshape(VyVdotA, [Nt, 1, Nag]) + ...
		Vy.*Ax.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;

	d2rdxdx(:, vy_jj, vx_jj) = permute(d2rdxdx(:, vx_jj, vy_jj), [1, 3, 2]);
	d2rdxdx(:, vy_jj, vy_jj) = AyVdotA.*reshape(AyVdotA, [Nt, 1, Nag]) + ...
		Ay.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;
	d2rdxdx(:, vy_jj, ax_jj) = AyVdotA.*reshape(VxVdotA, [Nt, 1, Nag]) + ...
		Vx.*Ay.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;
	d2rdxdx(:, vy_jj, ay_jj) = AyVdotA.*reshape(VyVdotA, [Nt, 1, Nag]) + ...
		(Vx.*Ax + 2*Vy.*Ay).*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;

	d2rdxdx(:, ax_jj, vx_jj) = permute(d2rdxdx(:, vx_jj, ax_jj), [1, 3, 2]);
	d2rdxdx(:, ax_jj, vy_jj) = permute(d2rdxdx(:, vy_jj, ax_jj), [1, 3, 2]);
	d2rdxdx(:, ax_jj, ax_jj) = VxVdotA.*reshape(VxVdotA, [Nt, 1, Nag]) + ...
		Vx.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;
	d2rdxdx(:, ax_jj, ay_jj) = VxVdotA.*reshape(VyVdotA, [Nt, 1, Nag]) + ...
		Vx.*Vy.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;

	d2rdxdx(:, ay_jj, vx_jj) = permute(d2rdxdx(:, vx_jj, ay_jj), [1, 3, 2]);
	d2rdxdx(:, ay_jj, vy_jj) = permute(d2rdxdx(:, vy_jj, ay_jj), [1, 3, 2]);
	d2rdxdx(:, ay_jj, ax_jj) = permute(d2rdxdx(:, ax_jj, ay_jj), [1, 3, 2]);
	d2rdxdx(:, ay_jj, ay_jj) = VyVdotA.*reshape(VyVdotA, [Nt, 1, Nag]) + ...
		Vy.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;
    %disp(reward.type)
end