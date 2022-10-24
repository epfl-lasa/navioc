function [r, g, drdu, d2rdudu, drdx, d2rdxdx, gfull, Hfull] = ...
	aleft4sumevalreward(reward, mdp_data, x, u, states, A, B, dxdu, d2xdudu)

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
r1 = 0.5*sum(VcrossA.^2, 2);
r = 0.5*r1.^2;

if nargout >= 2
	drdx = zeros(Nt, Nx);
	AyVcrossA = Ay.*VcrossA;
	mAxVcrossA = -Ax.*VcrossA;
	mVyVcrossA = -Vy.*VcrossA;
	VxVcrossA = Vx.*VcrossA;
	drdx(:, vx_jj) = AyVcrossA.*r1;
	drdx(:, vy_jj) = mAxVcrossA.*r1;
	drdx(:, ax_jj) = mVyVcrossA.*r1;
	drdx(:, ay_jj) = VxVcrossA.*r1;
	g = permute(gradprod(A,B,permute(drdx,[1 3 2])),[1 3 2]);
end
if nargout >= 3
	drdu = zeros(Nt, Nu);
	d2rdudu = zeros(Nt, Nu, Nu);
end
if nargout >= 6
	d2rdxdx = zeros(Nt, Nx, Nx);

	d2rdxdx(:, vx_jj, vx_jj) = AyVcrossA.*reshape(AyVcrossA, [Nt, 1, Nag]) + ...
		Ay.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;
	d2rdxdx(:, vx_jj, vy_jj) = AyVcrossA.*reshape(mAxVcrossA, [Nt, 1, Nag]) + ...
		-Ay.*Ax.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;
	d2rdxdx(:, vx_jj, ax_jj) = AyVcrossA.*reshape(mVyVcrossA, [Nt, 1, Nag]) + ... 
		-Ay.*Vy.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;
	d2rdxdx(:, vx_jj, ay_jj) = AyVcrossA.*reshape(VxVcrossA, [Nt, 1, Nag]) + ... 
		(2*Vx.*Ay - Vy.*Ax).*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;

	d2rdxdx(:, vy_jj, vx_jj) = permute(d2rdxdx(:, vx_jj, vy_jj), [1, 3, 2]);
	d2rdxdx(:, vy_jj, vy_jj) = mAxVcrossA.*reshape(mAxVcrossA, [Nt, 1, Nag]) + ... 
		Ax.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;
	d2rdxdx(:, vy_jj, ax_jj) = mAxVcrossA.*reshape(mVyVcrossA, [Nt, 1, Nag]) + ... 
		(2*Vy.*Ax - Vx.*Ay).*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;
	d2rdxdx(:, vy_jj, ay_jj) = mAxVcrossA.*reshape(VxVcrossA, [Nt, 1, Nag]) + ... 
		-Ax.*Vx.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;

	d2rdxdx(:, ax_jj, vx_jj) = permute(d2rdxdx(:, vx_jj, ax_jj), [1, 3, 2]);
	d2rdxdx(:, ax_jj, vy_jj) = permute(d2rdxdx(:, vy_jj, ax_jj), [1, 3, 2]);
	d2rdxdx(:, ax_jj, ax_jj) = mVyVcrossA.*reshape(mVyVcrossA, [Nt, 1, Nag]) + ... 
		Vy.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;
	d2rdxdx(:, ax_jj, ay_jj) = mVyVcrossA.*reshape(VxVcrossA, [Nt, 1, Nag]) + ... 
		-Vx.*Vy.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;

	d2rdxdx(:, ay_jj, vx_jj) = permute(d2rdxdx(:, vx_jj, ay_jj), [1, 3, 2]);
	d2rdxdx(:, ay_jj, vy_jj) = permute(d2rdxdx(:, vy_jj, ay_jj), [1, 3, 2]);
	d2rdxdx(:, ay_jj, ax_jj) = permute(d2rdxdx(:, ax_jj, ay_jj), [1, 3, 2]);
	d2rdxdx(:, ay_jj, ay_jj) = VxVcrossA.*reshape(VxVcrossA, [Nt, 1, Nag]) + ... 
		Vx.^2.*repmat(reshape(eye(Nag), [1, Nag, Nag]), [Nt, 1, 1]).*r1;
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