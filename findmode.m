function x_max = findmode(X, x_lim, N_x)

dx = (x_lim(2) - x_lim(1))/N_x;
cx = x_lim(1) + dx*((1:N_x) - 0.5);

J = floor((X - x_lim(1))/dx) + 1;
J(J > N_x | J < 1) = N_x + 1;

m = size(X, 1);
Indicators = zeros(m, N_x + 1);
Indicators((1:m)' + m*(J-1)) = 1;

Counts = sum(Indicators, 1);
[~, k_max] = max(Counts(1, 1:N_x));
x_max = cx(k_max);