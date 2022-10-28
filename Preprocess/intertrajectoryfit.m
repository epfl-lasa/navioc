function result = intertrajectoryfit(X, T, T_opt, h, s, XV_opt_1)

T_opt = reshape(T_opt, [length(T_opt), 1]);

if T(1) < T_opt(1)
	n_prepend = floor((T_opt(1) - T(1))/h) + 1;
	T_opt = [T_opt(1) + (-n_prepend:-1)'*h; T_opt];
else
	n_prepend = 0;
end

if T(end) > T_opt(end)
	n_append = floor((T(end) - T_opt(end))/h) + 1;
	T_opt = [T_opt; T_opt(end) + (1:n_append)'*h];
else
	n_append = 0;
end

N = length(T_opt);

[n, d] = size(X);

x_obs = reshape(X', [n*d, 1]);

% Let xi = [x_1; v_1; u_1; ... x_N; v_N; u_N]
% denote the vector of optimization variables.

% Let x_obs = [X(1,1); X(1,2); ... X(n,1); X(n,2)].

% Let x = P*xi estimate the true values corressponding to x_obs,
% i.e. x + noise = x_obs, with P of size (n*d) x (N*d*3).

P = zeros(n*d, N*d*3);
j = N - 1;
for i = n:-1:1
	while T_opt(j) > T(i)
		j = j - 1;
	end
	dt = T(i) - T_opt(j);
	[Ax, Bx] = discrete_time_system(dt, d, true);
	% x_obs{i} = Ax*[x,v]{j} + Bx*u{j+1}
	ii = (d*(i-1)+1):(d*i);
	P(ii, (3*d*(j-1)+1):(3*d*(j-1)+d*2)) = Ax;
	P(ii, (3*d*j+d*2+1):(3*d*(j+1))) = Bx;
end

[A, B] = discrete_time_system(h, d, false);

% The state trajectory needs to satisfy the assumed dynamics
% A*[x; v]{j} + B*u{j+1} - [x; v]{j+1} = 0, for j = 1, ... N-1
% This set of equality constraints is written as
%	G*xi = 0 = g
% with G of size ((N-1)*d*2) x (N*d*3).

n_g = (N-1)*d*2;
if nargin == 6 % constrain state at original T_opt(1)
	n_g = n_g + d*2;
end

G = zeros(n_g, N*d*3);
for j = 1:(N-1)
	ii = ((j - 1)*d*2 + 1):(j*d*2);
	G(ii, (3*d*(j-1)+1):(3*d*(j-1)+d*2)) = A;
	G(ii, (3*d*j+1):(3*d*j+d*2)) = -eye(2*d);
	G(ii, (3*d*j+d*2+1):(3*d*(j+1))) = B;
end
g = zeros(n_g, 1);

% constrain state at original T_opt(1) to be equal to given XV_opt_1
if nargin == 6
	G((n_g - d*2 + 1):end, (1:(d*2))+n_prepend*d*3) = eye(d*2);
	g((n_g - d*2 + 1):end) = XV_opt_1;
end

% The objective is expressed as
%	J = 0.5*xi^T H xi + f^T xi + const
% let u = [u_1; ... u_N] = Q xi.
%	J = ||P xi - x_obs||^2 + s ||Q xi||^2
%	  = (P xi - x_obs)^T (P xi - x_obs) + s (Q xi)^T (Q xi)
%	  = xi^T P^T P xi - xi^T P^T x_obs - x_obs^T P xi + x_obs^T x_obs + s xi^T Q^T Q xi
%	  = xi^T (P^T P + s Q^T Q) xi - 2 x_obs^T P xi + const
% Thus,
%	H = P^T P + s Q^T Q
%	f = - P^T x_obs

l = d*3;

P_opt = zeros(N*d, N*l); % P_opt*Xi = [x_1; ... x_N]
Q = zeros(N*d, N*l); % Q*Xi = [u_1; ... u_N]
R = zeros(N*d, N*l); % R*Xi = [v_1; ... v_N]
for t = 1:N
	ii = (d*(t - 1) + 1):(d*t);
	jjx = (l*(t - 1) + 1):(l*(t - 1) + d);
    jjv = (l*(t - 1) + d + 1):(l*(t - 1) + 2*d);
	jju = (l*(t - 1) + 2*d + 1):(l*t);
	P_opt(ii, jjx) = eye(d);
	Q(ii, jju) = eye(d);
    R(ii, jjv) = eye(d);
end

H = P'*P + s*Q'*Q;
f = -P'*x_obs;

options = optimoptions(@quadprog,'Display','none');

Xi = quadprog(H, f, [], [], G, g, [], [], [], options);

X_est = reshape(P*Xi, [d, n])';
X_opt = reshape(P_opt*Xi, [d, N])';
U_opt = reshape(Q*Xi, [d, N])';
V_opt = reshape(R*Xi, [d, N])';

if (n_append > 0) || (n_prepend > 0)
	T_opt = T_opt((1+n_prepend):(end-n_append));
	X_opt = X_opt((1+n_prepend):(end-n_append), :);
	U_opt = U_opt((1+n_prepend):(end-n_append), :);
	V_opt = V_opt((1+n_prepend):(end-n_append), :);
end

D = sqrt(sum((X_est - X).^2, 2));
averageDist = mean(D);
maxDist = max(D);

Accel = sqrt(sum(U_opt.^2, 2));
averageAccel = mean(Accel);
maxAccel = max(Accel);

result = struct(...
	'X_est', X_est, ...
	'T_opt', T_opt, ...
	'X_opt', X_opt, ...
	'V_opt', V_opt, ...
	'U_opt', U_opt, ...
	'averageDist', averageDist, 'maxDist', maxDist, ...
    'averageAccel', averageAccel, 'maxAccel', maxAccel);


function [A, B] = discrete_time_system(h, d, only_pos)
if ~only_pos
	A = [...
		eye(d), h*eye(d); ...
		zeros(d), eye(d) ... 
	];
	B = [...
		eye(d)*h^2/2; ...
		h*eye(d) ...
	];
else
	A = [eye(d), h*eye(d)];
	B = [eye(d)*h^2/2];
end