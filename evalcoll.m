function C = evalcoll(samples, R)
n_pairs_tot = 0;
for i = 1:length(samples)
	n_agents = length(samples{i}.s)/4;
	n_pairs_tot = n_pairs_tot + n_agents*(n_agents - 1)/2;
end
D = zeros(size(samples{1}.states, 1), n_pairs_tot);
l = 0;
for i = 1:length(samples)
	n_agents = length(samples{i}.s)/4;
	for j = 1:n_agents
		for k = (j + 1):n_agents
			l = l + 1;
			P_j = samples{i}.states(:, (2*j - 1):(2*j));
			P_k = samples{i}.states(:, (2*k - 1):(2*k));
			D(:, l) = sqrt(sum((P_j - P_k).^2, 2));
		end
	end
end

I = D < R;
start_mask = ~[zeros(1, n_pairs_tot); I(1:(end - 1), :)];
%end_mask = ~[I(2:end, :); zeros(1, n_pairs_tot)];
start_ind = I & start_mask;
%end_ind = I & end_mask;
C = sum(start_ind, 1);