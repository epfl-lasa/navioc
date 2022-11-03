function [D, D_gt] = evaldistrelpos(samples, re_samples)
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

			P_j_re = re_samples{i}.states(:, (2*j - 1):(2*j));
			P_k_re = re_samples{i}.states(:, (2*k - 1):(2*k));
			P_jk_re = P_j_re - P_k_re;
			
			D(:, l) = sqrt(sum((P_jk_re - P_jk_gt).^2, 2));
		end
	end
end