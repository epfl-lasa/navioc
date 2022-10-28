function bins = bindata(tracks, dt_bin)

tMin = inf;
tMax = -inf;
for k = 1:length(tracks)
	tMin = min([tMin, tracks{k}.t(1)]);
	tMax = max([tMax, tracks{k}.t(end)]);
end

n_bins = (tMax - tMin)/dt_bin;
if n_bins > floor(n_bins)
	n_bins = floor(n_bins) + 1;
else
	n_bins = floor(n_bins);
end

bin_counts = zeros(1, n_bins);

for k = 1:length(tracks)
	iMin = 1 + floor((tracks{k}.t(1) - tMin)/dt_bin);
	iMax = 1 + floor((tracks{k}.t(end) - tMin)/dt_bin);
	bin_counts(iMin:iMax) = bin_counts(iMin:iMax) + 1;
end

bin_registries = cell(1, n_bins);
for i = 1:n_bins
	bin_registries{i} = zeros(1, bin_counts(i));
	bin_counts(i) = 0;
end

for k = 1:length(tracks)
	if ~isempty(tracks{k})
		iMin = 1 + floor((tracks{k}.t(1) - tMin)/dt_bin);
		iMax = 1 + floor((tracks{k}.t(end) - tMin)/dt_bin);
		for i = iMin:iMax
			bin_counts(i) = bin_counts(i) + 1;
			bin_registries{i}(bin_counts(i)) = k;
		end
	end
end

bins = struct('t_min', tMin, 'dt_bin', dt_bin, 'registries', {bin_registries});