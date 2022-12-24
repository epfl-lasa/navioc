function verifyfeature(reward, mdp_data, x, u, dx, du, is_control_feature)

if nargin < 7
	is_control_feature = false;
end

[states, A, B] = crowdworldcontrol(mdp_data, x, u);

f = strcat(reward.type, 'evalreward');

[r, g, drdu, d2rdudu, drdx, d2rdxdx] = feval(f, ...
	reward, mdp_data, x, u, states, A, B);

if is_control_feature
	drdx_fd = zeros(size(drdx));
	d2rdxdx_fd = zeros(size(d2rdxdx));
	for j = 1:size(drdx, 2)
		dX = zeros(size(states));
		dX(:, j) = dx;
		[r_m, ~, ~, ~, drdx_m] = feval(f, reward, mdp_data, x, u, states - dX, A, B);
		[r_p, ~, ~, ~, drdx_p] = feval(f, reward, mdp_data, x, u, states + dX, A, B);
		drdx_fd(:, j) = (r_p - r_m)/2/dx;
		d2rdxdx_fd(:, :, j) = (drdx_p - drdx_m)/2/dx;
	end

	disp('drdx - absolute error')
	disp(abs_error(drdx, drdx_fd))
	disp('drdx - relative error')
	disp(rel_error(drdx, drdx_fd))
	disp('d2rdxdx - absolute error')
	disp(abs_error(d2rdxdx, d2rdxdx_fd))
	disp('d2rdxdx - relative error')
	disp(rel_error(d2rdxdx, d2rdxdx_fd))
else
	drdu_fd = zeros(size(drdu));
	d2rdudu_fd = zeros(size(d2rdudu));
	for j = 1:size(drdu, 2)
		dU = zeros(size(u));
		dU(:, j) = du;
		[r_m, g_m, drdu_m, d2rdudu_m] = feval(f, reward, mdp_data, x, u - dU, states, A, B);
		[r_p, g_p, drdu_p, d2rdudu_p] = feval(f, reward, mdp_data, x, u + dU, states, A, B);
		drdu_fd(:, j) = (r_p - r_m)/2/dx;
		d2rdudu_fd(:, :, j) = (drdu_p - drdu_m)/2/du;
	end
	disp('drdu - absolute error')
	disp(abs_error(drdu, drdu_fd))
	disp('drdu - relative error')
	disp(rel_error(drdu, drdu_fd))
	disp('d2rdudu - absolute error')
	disp(abs_error(d2rdudu, d2rdudu_fd))
	disp('d2rdudu - relative error')
	disp(rel_error(d2rdudu, d2rdudu_fd))
end

function err = abs_error(a, b)
d = reshape(a - b, [numel(a), 1]);
err = max(abs(d));
% if length(size(a)) == 3
% 	disp('Diag')
% 	disp(reshape(max(abs(a - b), [], 1), [size(a, 2), size(a, 2)]))
% 	disp('end')
% end

function err = rel_error(a, b)
a_ = reshape(a, [numel(a), 1]);
b_ = reshape(b, [numel(a), 1]);
err = max(abs(a_(a_ ~= 0) - b_(a_ ~= 0))./abs(a_(a_ ~= 0)));