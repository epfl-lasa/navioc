function Phi_ref = estphiref(track_fit, y_1, y_2, dx)
Phi_ref = zeros(length(track_fit.T_opt), 1);

ind_corr = track_fit.X_opt(:, 2) > y_1 & track_fit.X_opt(:, 2) < y_2;
start_mask = ~[false; ind_corr(1:(end-1))];
end_mask = ~[ind_corr(2:end); false];
start_ind = ind_corr & start_mask;
end_ind = ind_corr & end_mask;

ii_segments = [find(start_ind), find(end_ind)];
% iterate over segments fully inside the horizontal corridor
for j = 1:size(ii_segments, 1)
	ii_seg = ii_segments(j, 1):ii_segments(j, 2);
	Xx_seg = track_fit.X_opt(ii_seg, 1);
	if (max(Xx_seg) - min(Xx_seg) > dx || ...
		(ind_corr(1) && j == 1) || ...
		(ind_corr(end) && j == size(ii_segments, 1)))
		% horizontal motion
		Vx_seg = track_fit.V_opt(ii_seg, 1);
		Phi_ref_seg = zeros(length(ii_seg), 1); % right
		Phi_ref_seg(Vx_seg < 0) = pi; % left
	else
		% vertical motion
		Vy_seg = track_fit.V_opt(ii_seg, 2);
		Phi_ref_seg = ones(length(ii_seg), 1)*pi/2; % up
		Phi_ref_seg(Vy_seg < 0) = -pi/2; % down
	end
	Phi_ref(ii_seg) = Phi_ref_seg;
end
% outside horizontal corridor, choose phi_ref from {-pi/2, pi/2}
Phi_ref(~ind_corr) = -pi/2;
Phi_ref(~ind_corr & track_fit.V_opt(:, 2) > 0) = pi/2;