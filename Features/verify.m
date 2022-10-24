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
err = max(abs(a - b), [], 'all');

function err = rel_error(a, b)
err = max(abs(a(a ~= 0) - b(a ~= 0))./abs(a(a ~= 0)), [], 'all');