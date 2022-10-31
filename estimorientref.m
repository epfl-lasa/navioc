function Phi_ref = estimorientref(X_opt, y_1, y_2, dx_min)

above_y_1 = X_opt(:, 2) > y_1;
above_y_2 = X_opt(:, 2) > y_2;
cross_y_1 = above_y_1(1:(end - 1)) ~= above_y_1(2:end);
cross_y_2 = above_y_2(1:(end - 1)) ~= above_y_2(2:end);

Phi_ref = zeros(size(X_opt, 1), 1);
if X_opt(1, 2) > y_2 && X_opt(end, 2) < y_1
	ind_horicorr = X_opt(:, 2) > y_1 & X_opt(:, 2) < y_2;
	ii_horicorr = find(ind_horicorr);
	dx_in_out = X_opt(ii_horicorr(end), 1) - X_opt(ii_horicorr(1), 1);
	if dx_in_out > dx_min
		Phi_ref(ind_horicorr) = 0; % right inside horizontal corridor
		Phi_ref(~ind_horicorr) = -pi/2; % straight down
	elseif dx_in_out < -dx_min
		Phi_ref(ind_horicorr) = -pi; % left inside horizontal corridor
		Phi_ref(~ind_horicorr) = -pi/2; % straight down
	else
		Phi_ref(:) = -pi/2; % straight down
	end
elseif X_opt(1, 2) < y_1 && X_opt(end, 2) > y_2
	ind_horicorr = X_opt(:, 2) > y_1 & X_opt(:, 2) < y_2;
	ii_horicorr = find(ind_horicorr);
	dx_in_out = X_opt(ii_horicorr(end), 1) - X_opt(ii_horicorr(1), 1);
	if dx_in_out > dx_min
		Phi_ref(ind_horicorr) = 0; % right inside horizontal corridor
		Phi_ref(~ind_horicorr) = pi/2; % straight up
	elseif dx_in_out < -dx_min
		Phi_ref(ind_horicorr) = -pi; % left inside horizontal corridor
		Phi_ref(~ind_horicorr) = pi/2; % straight up
	else
		Phi_ref(:) = pi/2; % straight up
	end
elseif all(X_opt([1, end], 2) <= y_2 & X_opt([1, end], 2) >= y_1)
	% straight horizontal
	if X_opt(1, 1) < X_opt(end, 1)
		Phi_ref(:) = 0; % right
		% is there a turning point left-right?
		[x_min, i_min] = min(X_opt(:, 1));
		if X_opt(1, 1) - x_min > dx_min/2
			Phi_ref(1:i_min) = -pi; % left
		end
	else
		Phi_ref(:) = -pi; % left
		% is there a turning point right-left?
		[x_max, i_max] = max(X_opt(:, 1));
		if x_max - X_opt(1, 1) > dx_min/2
			Phi_ref(1:i_max) = 0; % right
		end
	end
elseif X_opt(1, 2) > y_2 && X_opt(end, 2) < y_2
	% down-horizontal
	if X_opt(1, 1) < X_opt(end, 1)
		% down-right
		Phi_ref(X_opt(:, 2) >= y_2) = -pi/2;
		Phi_ref(X_opt(:, 2) < y_2) = 0;
	else
		% down-left
		Phi_ref(X_opt(:, 2) >= y_2) = -pi/2;
		Phi_ref(X_opt(:, 2) < y_2) = -pi;
	end
elseif X_opt(1, 2) < y_1 && X_opt(end, 2) < y_2
	% up-horizontal
	if X_opt(1, 1) < X_opt(end, 1)
		% up-right
		Phi_ref(X_opt(:, 2) <= y_1) = pi/2;
		Phi_ref(X_opt(:, 2) > y_1) = 0;
	else
		% up-left
		Phi_ref(X_opt(:, 2) <= y_1) = pi/2;
		Phi_ref(X_opt(:, 2) > y_1) = -pi;
	end
elseif X_opt(1, 2) < y_2 && X_opt(end, 2) > y_2
	% horizontal-up
	if X_opt(1, 1) < X_opt(end, 1)
		% right-up
		Phi_ref(X_opt(:, 2) <= y_2) = 0;
		Phi_ref(X_opt(:, 2) > y_2) = pi/2;
	else
		% left-up
		Phi_ref(X_opt(:, 2) <= y_2) = -pi;
		Phi_ref(X_opt(:, 2) > y_2) = pi/2;
	end
elseif X_opt(1, 2) < y_2 && X_opt(end, 2) < y_1
	% horizontal-down
	if X_opt(1, 1) < X_opt(end, 1)
		% right-down
		Phi_ref(X_opt(:, 2) >= y_1) = 0;
		Phi_ref(X_opt(:, 2) < y_1) = -pi/2;
	else
		% left-down
		Phi_ref(X_opt(:, 2) >= y_1) = -pi;
		Phi_ref(X_opt(:, 2) < y_1) = -pi/2;
	end
end