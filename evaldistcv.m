function D = evaldistcv(samples, h, speed_min)
D = [];
for i = 1:length(samples)
	n_agents = length(samples{i}.s)/4;
	for j = 1:n_agents
		V_gt = samples{i}.states(:, 2*n_agents + ((2*j - 1):(2*j)));
		Vmag_gt = sqrt(sum(V_gt.^2, 2));
		if any(Vmag_gt > speed_min)
			P_gt = samples{i}.states(:, (2*j - 1):(2*j));
			p_0 = samples{i}.s((2*j - 1):(2*j));
			v_0 = samples{i}.s(2*n_agents + ((2*j - 1):(2*j)));
			P_cv = p_0 + v_0.*(1:size(P_gt, 1))'*h;
			D = [D, sqrt(sum((P_gt - P_cv).^2, 2))];
		end
	end
end