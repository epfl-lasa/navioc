function plotmeanstd(X, Y, c, s)
	Mu = mean(Y, 2);
	Sigma = std(Y, 0, 2);
	plot(X, Mu, c, 'DisplayName', s)
	plot(X, Mu + Sigma, c, 'HandleVisibility', 'off')
	plot(X, Mu - Sigma, c, 'HandleVisibility', 'off')
end