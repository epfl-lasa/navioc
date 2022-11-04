function [speed_ref, i_max] = estimspeedandgoal(track_fit, goals, speed_min, speed_max, n_bins)

Vmag = sqrt(sum(track_fit.V_opt.^2, 2));

speed_ref = estspeedref(Vmag, speed_min, speed_max, n_bins);

ind = Vmag > speed_min;
E_vel = track_fit.V_opt(ind, :)./Vmag(ind);
P = track_fit.X_opt(ind, :);

scores = zeros(size(goals, 1), 1);
E_goal = cell(1, size(goals, 1));
for i = 1:size(goals, 1)
	E_goal{i} = goals(i, :) - P;
	E_goal{i} = E_goal{i}./sqrt(sum(E_goal{i}.^2, 2));
	cosDPhi = sum(E_goal{i}.*E_vel, 2);
	scores(i) = median(cosDPhi((1 + floor(length(cosDPhi)/2)):end));
end
[~, i_max] = max(scores);
%Vref = E_goal{i_max}*speed_ref;

function speed_ref = estspeedref(Vmag, speed_min, speed_max, n_bins)
Vmag_motion = Vmag(Vmag >= speed_min);

if ~isempty(Vmag_motion)
	dspeed = (speed_max - speed_min)/n_bins;
	edges = speed_min + (0:n_bins)*dspeed;
	[counts, ~] = histcounts(Vmag_motion, edges);

	[~, j_max] = max(counts);

	speed_ref = speed_min + (j_max - 0.5)*dspeed;
else
	speed_ref = 0;
end