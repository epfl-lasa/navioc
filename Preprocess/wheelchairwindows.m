function [windows, slice_indices] = wheelchairwindows(tracks, dt_margin)

bins = bindata(tracks, 20.0);

statistics = crowdstatistics(tracks, bins, 1.0);

indicator = logical(zeros(size(statistics.n_agents_over_t(1, :))));

for i = 1:length(statistics.agent_types)
	if (strcmp(statistics.agent_types(i), "manual_wheelchair") || ...
		strcmp(statistics.agent_types(i), "powered_wheelchair"))
		indicator = indicator | (statistics.n_agents_over_t(i, :) > 0);
	end
end

start_mask = ~[false, indicator(1:(end-1))];
end_mask = ~[indicator(2:end), false];
start_ind = indicator & start_mask;
end_ind = indicator & end_mask;
j_windows = [find(start_ind); find(end_ind)];
windows = zeros(size(j_windows));
slice_indices = cell(1, size(j_windows, 2));
for j = 1:size(j_windows, 2)
	t1 = statistics.t(j_windows(1, j)) - dt_margin;
	t2 = statistics.t(j_windows(2, j)) + dt_margin;
	windows(:, j) = [t1; t2];
	slice_indices{j} = slicebins(bins, t1, t2);
end