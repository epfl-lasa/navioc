function D = evaldist(samples, re_samples)
	D = [];
	for i = 1:length(samples)
		n_agents = length(samples{i}.s)/4;
		for j = 1:n_agents
			P_gt = samples{i}.states(:, (2*j - 1):(2*j));
			P_re = re_samples{i}.states(:, (2*j - 1):(2*j));
			D = [D, sqrt(sum((P_gt - P_re).^2, 2))];
	end
end