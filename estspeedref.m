function speed_ref = estspeedref(Vmag, speed_min, speed_max, n_bins)

%Vmag = sqrt(sum(V_opt.^2, 2));
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
