function D = evaldistcv(samples, h)
	D = [];
	for i = 1:length(samples)
		n_agents = length(samples{i}.s)/4;
		for j = 1:n_agents
			P_gt = samples{i}.states(:, (2*j - 1):(2*j));
			p_0 = samples{i}.s((2*j - 1):(2*j));
			v_0 = samples{i}.s(2*n_agents + ((2*j - 1):(2*j)));
			P_cv = p_0 + v_0.*(1:size(P_gt, 1))'*h;
			D = [D, sqrt(sum((P_gt - P_cv).^2, 2))];
	end
end