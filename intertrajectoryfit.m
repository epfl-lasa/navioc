function result = intertrajectoryfit(X, T, T_opt, h, s, const_jerk_dynamics, XVA_opt_1)

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

% if T(1) < T_opt(1)
% 	T_opt = [T(1); T_opt];
% 	PREPEND = true;
% else
% 	PREPEND = false;
% end

% if T(end) > T_opt(end)
% 	T_opt = [T_opt; T(end)];
% 	APPEND = true;
% else
% 	APPEND = false;
% end

N = length(T_opt);

[n, d] = size(X);

x_obs = reshape(X', [n*d, 1]);

% Let xi = [x_1; v_1; a_1; u_1; ... x_N; v_N; a_N; u_N]
% denote the vector of optimization variables.

% Let x_obs = [X(1,1); X(1,2); ... X(n,1); X(n,2)].

% Let x = P*xi estimate the true values corressponding to x_obs,
% i.e. x + noise = x_obs, with P of size (n*d) x (N*d*4).

P = zeros(n*d, N*d*4);
j = N - 1;
for i = n:-1:1
	while T_opt(j) > T(i)
		j = j - 1;
	end
	dt = T(i) - T_opt(j);
	[Ax, Bx] = discrete_time_system(dt, d, const_jerk_dynamics, true);
	% x_obs{i} = Ax*[x,v,a]{j} + Bx*u{j+1}
	ii = (d*(i-1)+1):(d*i);
	P(ii, (4*d*(j-1)+1):(4*d*(j-1)+d*3)) = Ax;
	P(ii, (4*d*j+d*3+1):(4*d*(j+1))) = Bx;
end

[A, B] = discrete_time_system(h, d, const_jerk_dynamics, false);

% The state trajectory needs to satisfy the assumed dynamics
% A*[x; v; a]{j} + B*u{j+1} - [x; v; a]{j+1} = 0, for j = 1, ... N-1
% This set of equality constraints is written as
%	G*xi = 0 = g
% with G of size ((N-1)*d*3) x (N*d*4).

n_g = (N-1)*d*3;
if nargin == 7
	n_g = n_g + d*3;
end

G = zeros(n_g, N*d*4);
for j = 1:(N-1)
	ii = ((j - 1)*d*3 + 1):(j*d*3);
	G(ii, (4*d*(j-1)+1):(4*d*(j-1)+d*3)) = A;
	G(ii, (4*d*j+1):(4*d*j+d*3)) = -eye(3*d);
	G(ii, (4*d*j+d*3+1):(4*d*(j+1))) = B;
end
g = zeros(n_g, 1);

% constrain state at original T_opt(1)
if nargin == 7
	G((n_g - d*3 + 1):end, (1:(d*3))+n_prepend*d*4) = eye(d*3);
	g((n_g - d*3 + 1):end) = XVA_opt_1;
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

m = d*3;
l = d*4;

P_opt = zeros(N*d, N*l);
Q = zeros(N*d, N*l);
R = zeros(N*d, N*l);
W = zeros(N*d, N*l);
for t = 1:N
	ii = (d*(t - 1) + 1):(d*t);
	jjx = (l*(t - 1) + 1):(l*(t - 1) + d);
    jjv = (l*(t - 1) + d + 1):(l*(t - 1) + 2*d);
    jja = (l*(t - 1) + 2*d + 1):(l*(t - 1) + 3*d);
	jju = (l*(t - 1) + 3*d + 1):(l*t);
	P_opt(ii, jjx) = eye(d);
	Q(ii, jju) = eye(d);
    R(ii, jjv) = eye(d);
    W(ii, jja) = eye(d);
end

if length(s) == 1
    H = P'*P + s*Q'*Q;
else
    H = P'*P + s(1)*Q'*Q + s(2)*W'*W + s(3)*R'*R;
end
f = -P'*x_obs;

options = optimoptions(@quadprog,'Display','none');

Xi = quadprog(H, f, [], [], G, g, [], [], [], options);

X_est = reshape(P*Xi, [d, n])';
X_opt = reshape(P_opt*Xi, [d, N])';
U_opt = reshape(Q*Xi, [d, N])';
V_opt = reshape(R*Xi, [d, N])';
A_opt = reshape(W*Xi, [d, N])';

if (n_append > 0) || (n_prepend > 0)
	T_opt = T_opt((1+n_prepend):(end-n_append));
	X_opt = X_opt((1+n_prepend):(end-n_append), :);
	U_opt = U_opt((1+n_prepend):(end-n_append), :);
	V_opt = V_opt((1+n_prepend):(end-n_append), :);
	A_opt = A_opt((1+n_prepend):(end-n_append), :);
end

D = sqrt(sum((X_est - X).^2, 2));
averageDist = mean(D);
maxDist = max(D);

Jerk = sqrt(sum(U_opt.^2, 2));
averageJerk = mean(Jerk);
maxJerk = max(Jerk);

result = struct(...
	'X_est', X_est, ...
	'T_opt', T_opt, ...
	'X_opt', X_opt, ...
	'V_opt', V_opt, ...
	'A_opt', A_opt, ...
	'U_opt', U_opt, ...
	'averageDist', averageDist, 'maxDist', maxDist, ...
    'averageJerk', averageJerk, 'maxJerk', maxJerk);


function [A, B] = discrete_time_system(h, d, const_jerk_dynamics, only_pos)
if ~only_pos
	if ~const_jerk_dynamics
		A = [...
			eye(d), h*eye(d), zeros(d); ...
			zeros(d), eye(d), h*eye(d); ...
			zeros(d), zeros(d), eye(d)];
		B = [zeros(d); zeros(d); h*eye(d)];
	else
		A = [...
			eye(d), h*eye(d), eye(d)*h^2/2; ...
			zeros(d), eye(d), h*eye(d); ...
			zeros(d), zeros(d), eye(d)];
		B = [eye(d)*h^3/6; eye(d)*h^2/2; h*eye(d)];
	end
else
	if ~const_jerk_dynamics
		A = [eye(d), h*eye(d), zeros(d)];
		B = [zeros(d)];
	else
		A = [eye(d), h*eye(d), eye(d)*h^2/2];
		B = [eye(d)*h^3/6];
	end
end






%TRAJECTORYFIT Fits state-action-sequence to position data.
%   A sequence of states and actions is obtained by least squares
%   minimization of the squared difference between the estimated and observed
%   positions and the squared actions times a regularization constant, satisfying
%   the assumed linear time-independent triple integrator dynamics.
%   Formally, the function solves
%       min_{x{0:T},v{0:T},a{0:T},u{1:T}} ||x - x_observation||^2 + s*||u||^2
%       s.t. [x; v; a]{t+1} = A [x; v; a]{t} + B u{t+1}
%
%   INPUT ARGUMENTS
%   - X: N-by-d vector of observed d-dimensional positions
%   - h: time step between positions
%   - s: regularization constant to control smoothness
%
%   RETURN VALUES
%   - result: struct with fields
%       - X_opt: N-by-d vector of fit positions
%       - V_opt: N-by-d vector of fit velocities
%       - A_opt: N-by-d vector of fit accelerations
%       - U_opt: N-by-d vector of fit jerks
%       - averageDist: average distance between original and fit positions
%       - maxDist: maximum distance between original and fit positions
%       - averageJerk: fit jerk's average magnitude
%       - maxJerk: fit jerk's maximum magnitude