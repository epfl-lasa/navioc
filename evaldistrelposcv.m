function [D, D_gt] = evaldistrelposcv(samples, h)
n_pairs_tot = 0;
for i = 1:length(samples)
	n_agents = length(samples{i}.s)/4;
	n_pairs_tot = n_pairs_tot + n_agents*(n_agents - 1)/2;
end
D = zeros(size(samples{1}.states, 1), n_pairs_tot);
D_gt = D;
l = 0;
for i = 1:length(samples)
	n_agents = length(samples{i}.s)/4;
	for j = 1:n_agents
		for k = (j + 1):n_agents
			l = l + 1;

			P_j_gt = samples{i}.states(:, (2*j - 1):(2*j));
			P_k_gt = samples{i}.states(:, (2*k - 1):(2*k));
			P_jk_gt = P_j_gt - P_k_gt;
			D_gt(:, l) = sqrt(sum(P_jk_gt.^2, 2));

			p_0_j = samples{i}.s((2*j - 1):(2*j));
			v_0_j = samples{i}.s(2*n_agents + ((2*j - 1):(2*j)));
			P_cv_j = p_0_j + v_0_j.*(1:size(P_j_gt, 1))'*h;

			p_0_k = samples{i}.s((2*k - 1):(2*k));
			v_0_k = samples{i}.s(2*n_agents + ((2*k - 1):(2*k)));
			P_cv_k = p_0_k  + v_0_k .*(1:size(P_k_gt, 1))'*h;

			P_jk_cv = P_cv_j - P_cv_k;
			
			D(:, l) = sqrt(sum((P_jk_cv - P_jk_gt).^2, 2));
		end
	end
end