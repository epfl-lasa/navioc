function indices = slicebins(bins, t1, t2)

indices = [];

t1 = t1 - bins.t_min;
t2 = t2 - bins.t_min;

b1 = max([1 + floor(t1/bins.dt_bin), 1]);
b2 = min([1 + floor(t2/bins.dt_bin), length(bins.registries)]);

for b = b1:b2
	indices = sort(unique([indices, bins.registries{b}]));
end