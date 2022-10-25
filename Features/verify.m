function verify(reward, mdp_data, x, u, dx, du)

[states, A, B] = crowdworldcontrol(mdp_data, x, u);

f = strcat(reward.type, 'evalreward');

[r, g, drdu, d2rdudu, drdx, d2rdxdx] = feval(...
	f, reward, mdp_data, x, u, states, A, B);

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