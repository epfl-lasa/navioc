t_windows = [
39, 62,
30, 49,
40, 50,
39, 61,
40, 60,
38, 52
];

v_mag = [];
d_x = zeros(1, 6);
for i = 1:6
	mat = loadexp(getfpaths("crowd", "navioc", i));
	t_1 = mat.odom_sample.t(1) + t_windows(i, 1);
	t_2 = mat.odom_sample.t(1) + t_windows(i, 2);
	ind_odom = t_1 <= mat.odom_sample.t & mat.odom_sample.t <= t_2;
	v_mag = [v_mag; sqrt(sum(mat.odom_sample.states(ind_odom, 3:4).^2, 2))];
	d_x(i) = max(mat.odom_sample.states(ind_odom, 1)) - min(mat.odom_sample.states(ind_odom, 1));
end

d_t = t_windows(:, 2) - t_windows(:, 1);

fprintf('v_mag = %.2f +/- %.2f\n', mean(v_mag), std(v_mag))
fprintf('d_x = %.2f +/- %.2f\n', mean(d_x), std(d_x))
fprintf('d_t = %.2f +/- %.2f\n', mean(d_t), std(d_t))