function result = trackfit(track, t1, t2, h, s, N_max, dt_extra)
dt1 = track.t(1) - t1;
if dt1 >= 0
	n_off_1 = ceil(dt1/h);
	i_min = 1;
else
	n_off_1 = 0;
	i_min = find(track.t > t1, 1);
end
dt2 = t2 - track.t(end);
if dt2 >= 0
	n_off_2 = ceil(dt2/h);
	i_max = length(track.t);
else
	n_off_2 = 0;
	i_max = find(track.t < t2, 1, 'last');
end
if isempty(i_min) || isempty(i_max) || (t1 + n_off_1*h) > (t2 - n_off_2*h)
	result = struct('empty', true, 'type', track.type);
else
	T_opt = (t1 + n_off_1*h):h:(t2 - n_off_2*h);
	idx = max([1, i_min-1]):min([i_max + 1, length(track.t)]);
	X = [track.x(idx), track.y(idx)];
	T = track.t(idx);
	res = splitfit(X, T, T_opt, h, s, N_max, dt_extra);
	result = struct('empty', false, 'type', track.type, 'fit', res, 'n_off', n_off_1, 'T', T, 'X', X);
end