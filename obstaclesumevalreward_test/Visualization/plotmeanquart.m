function plotmeanquart(X, Y, c, s)
	Ys = sort(Y, 2);
	N = size(Ys, 2);
	lowQuart = Ys(:, floor(N/4));
	upQuart = Ys(:, ceil(N/4*3));
	plot(X, mean(Y, 2), 'Color', c, 'DisplayName', s)
	plot(X, median(Y, 2), 'Color', c, 'DisplayName', s, 'LineStyle', '-.', 'HandleVisibility', 'off')
	plot(X, upQuart, 'Color', c, 'HandleVisibility', 'off', 'LineStyle', '--')
	plot(X, lowQuart, 'Color', c, 'HandleVisibility', 'off', 'LineStyle', '--')
	%plot(X, max(Y, [], 2), c, 'HandleVisibility', 'off')
	%plot(X, min(Y, [], 2), c, 'HandleVisibility', 'off')
end