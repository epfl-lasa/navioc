function plotquartiles(X, Y, c)
	Ys = sort(Y, 2);
	N = size(Ys, 2);
	lowQuart = Ys(:, floor(N/4));
	upQuart = Ys(:, ceil(N/4*3));
	h = patch([X; flip(X)], [upQuart; flip(lowQuart)], c, 'HandleVisibility', 'off', 'LineStyle', 'None');
	set(h, 'FaceVertexAlphaData', 0.1);
	set(h, 'FaceAlpha', 'flat');
end