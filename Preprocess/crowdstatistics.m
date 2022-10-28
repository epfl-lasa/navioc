function result = crowdstatistics(tracks, bins, dt)

dt_bin = bins.dt_bin;
tMin = bins.t_min;
n_bins = length(bins.registries);
tMax = tMin + n_bins*dt_bin;

t = tMin:dt:tMax;
n_t = length(t);

agent_types = [
	"manual_wheelchair"
	"powered_wheelchair"
    "stroller"
    "ped"
%    "car"
%    "pepper_robot"
%    "bicycle"
%    "skateboard"
%    "bus"
];

n_types = length(agent_types);

n_agents_over_t = zeros(n_types, n_t);

for j = 1:n_t
	i = min([1 + floor((t(j) - tMin)/dt_bin), n_bins]);
	for k = bins.registries{i}
		if tracks{k}.t(end) > t(j) && tracks{k}.t(1) < t(j)
			for l = 1:n_types
				if strcmp(tracks{k}.type, agent_types(l))
					n_agents_over_t(l, j) = n_agents_over_t(l, j) + 1;
					break
				end
			end
		end
	end
end

result = struct('t', t, 'n_agents_over_t', n_agents_over_t, 'agent_types', agent_types);

% hold on
% for i = 1:n_types
% 	plot(t, n_agents_over_t(i, :));
% end
% legend(agent_types)