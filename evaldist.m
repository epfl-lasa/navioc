function D = evaldist(samples, re_samples, N_skip, speed_min)
if nargin < 3
	N_skip = 1;
end
if nargin < 4
	speed_min = 0.0;
end
D = [];
for i = 1:length(samples)
	n_agents = length(samples{i}.s)/4;
	for j = 1:n_agents
		V_gt = samples{i}.states(:, 2*n_agents + ((2*j - 1):(2*j)));
		Vmag_gt = sqrt(sum(V_gt.^2, 2));
		if any(Vmag_gt > speed_min)
			P_gt = samples{i}.states(:, (2*j - 1):(2*j));
			P_re = re_samples{i}.states(N_skip:N_skip:end, (2*j - 1):(2*j));
			D = [D, sqrt(sum((P_gt - P_re).^2, 2))];
		end
	end
end