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
d = [];
ie = [];
d_oc = [];
for i = 1:6
	mat = loadexp(getfpaths("crowd", "navioc", i));
	t_1 = mat.odom_sample.t(1) + t_windows(i, 1);
	t_2 = mat.odom_sample.t(1) + t_windows(i, 2);
	ind_odom = t_1 <= mat.odom_sample.t & mat.odom_sample.t <= t_2;
	v_mag = [v_mag; sqrt(sum(mat.odom_sample.states(ind_odom, 3:4).^2, 2))];
	d_x(i) = max(mat.odom_sample.states(ind_odom, 1)) - min(mat.odom_sample.states(ind_odom, 1));

	% TRACKED_PERSONS
	tracks_def = mat.tracked_persons.isdef;
	tracks_t = mat.tracked_persons.t;
	ind_tracks = tracks_t <= t_2 & tracks_t >= t_1;
	states_interp = interp1(mat.odom_sample.t, mat.odom_sample.states, tracks_t(ind_tracks));
	Px_rel = mat.tracked_persons.states(ind_tracks, :, 1) - states_interp(:, 1);
	Py_rel = mat.tracked_persons.states(ind_tracks, :, 2) - states_interp(:, 2);
	Vx_rel = mat.tracked_persons.states(ind_tracks, :, 3) - states_interp(:, 3);
	Vy_rel = mat.tracked_persons.states(ind_tracks, :, 4) - states_interp(:, 4);
	Px_rel_def = Px_rel(tracks_def(ind_tracks, :));
	Py_rel_def = Py_rel(tracks_def(ind_tracks, :));
	Vx_rel_def = Vx_rel(tracks_def(ind_tracks, :));
	Vy_rel_def = Vy_rel(tracks_def(ind_tracks, :));

	d = [d; sqrt(Px_rel_def.^2 + Py_rel_def.^2)];

	res = interactionmetrics([Px_rel_def, Py_rel_def], [Vx_rel_def, Vy_rel_def], 0.4, 3.0);
	ie = [ie; res.ie];

	% oc_reply
	ind_oc = t_1 <= mat.oc_reply.t & mat.oc_reply.t <= t_2;
	k = 2; % 0.4 s later
	dt = (k - 1)*0.4;
	states_interp_oc = interp1(mat.odom_sample.t, mat.odom_sample.states, mat.oc_reply.t(ind_oc)+dt);
	Px_oc = mat.oc_reply.states(ind_oc, k, 1);
	Py_oc = mat.oc_reply.states(ind_oc, k, 2);
	Px_rel_oc = Px_oc - states_interp_oc(:, 1);
	Py_rel_oc = Py_oc - states_interp_oc(:, 2);
	d_oc = [d_oc; sqrt(Px_rel_oc.^2 + Py_rel_oc.^2)];
end

d_t = t_windows(:, 2) - t_windows(:, 1);

fprintf('v_mag = %.2f +/- %.2f\n', mean(v_mag), std(v_mag))
fprintf('d_x = %.2f +/- %.2f\n', mean(d_x), std(d_x))
fprintf('d_t = %.2f +/- %.2f\n', mean(d_t), std(d_t))

figure
histogram(d)
figure
histogram(ie(ie > 0 & ie < 1))
figure
histogram(d_oc)





function result = interactionmetrics(P, V, R, tau0)
[MPD, TMPD] = minimum_predicted_distance(P, V);
TTC = time_to_collision(P, V, R);
IE = zeros(length(TTC), 1);
indPos = TTC > 0.0;
IE(indPos) = 1./TTC(indPos).^2.*exp(-TTC(indPos)/tau0);
IE(TTC == 0.0) = inf;
result = struct(...
	'd', sqrt(sum(P.^2, 2)), ...
	'mpd', MPD, ...
	'tmpd', TMPD, ...
	'ttc', TTC, ...
	'ie', IE, ...
	'approx_ie', simple_approximation(P, V, R, 10.0));
end

function [D, T] = minimum_predicted_distance(P, V)
V2 = sum(V.^2, 2);
P2 = sum(P.^2, 2);
PV = sum(P.*V, 2);
n = length(V2);
T = zeros(n, 1);
D = zeros(n, 1);
indVZero = V2 == 0.0;
indVPositive = ~indVZero;
T(indVZero) = nan;
D(indVZero) = sqrt(P2(indVZero));
T(indVPositive) = -PV(indVPositive)./V2(indVPositive);
P_MPD = P(indVPositive, :) + T(indVPositive).*V(indVPositive, :);
D(indVPositive) = sqrt(sum(P_MPD.^2, 2));
%D(indVPositive) = P2(indVPositive) + ...
%	T(indVPositive).^2.*V2(indVPositive) + ...
%	2*T(indVPositive).*PV(indVPositive);
end

function T = time_to_collision(P, V, R)
V2 = sum(V.^2, 2);
P2 = sum(P.^2, 2);
PV = sum(P.*V, 2);
R2 = R^2;
D = PV.^2 - V2.*(P2 - R2);
n = length(P2);
T = zeros(n, 1);
indSep = P2 > R2;
indVZero = V2 == 0.0;
indNoSol = D < 0.0;
indNeg = indSep & ~indVZero & ~indNoSol & (PV > 0);
indPos = indSep & ~indVZero & ~indNoSol & (PV < 0);
T(indSep & (indVZero | indNoSol)) = nan;
T(indNeg) = (-PV(indNeg) + sqrt(D(indNeg)))./V2(indNeg);
T(indPos) = (-PV(indPos) - sqrt(D(indPos)))./V2(indPos);
end

function IE = simple_approximation(P, V, R, s)
V2 = sum(V.^2, 2);
V_mag = sqrt(V2);
P2 = sum(P.^2, 2);
P_mag = sqrt(P2);
PV = sum(P.*V, 2);
X = -PV - P2.*V_mag./sqrt(P2 + R^2);
H = 1./(1 + exp(-s*X));
MPD2 = P2 - PV.^2./(V2 + 0.01);
IE = V2.*H./(0.22*R^2 + (P_mag - R).^2 + 2*(P_mag/R - 1).*MPD2);
end