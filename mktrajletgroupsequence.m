function [seq, t_0] = mktrajletgroupsequence(fit_batch, N_steps, h, t_min, t_max)
t_0 = inf;
%t_max = -inf;
n_off_0 = -1;
for i = 1:length(fit_batch.fits)
	if ~fit_batch.fits{i}.empty
		%t_max = max([t_max, fit_batch{i}.fit.T_opt(1)]);
		if t_0 > fit_batch.fits{i}.fit.T_opt(1)
			t_0 = fit_batch.fits{i}.fit.T_opt(1);
			n_off_0 = fit_batch.fits{i}.n_off;
		end
	end
end
seq = {};
j = 1;
while true
	t_j = t_0 + (j - 1)*h;
	if t_j < t_min
		continue
	elseif t_j + (N_steps-1)*h > t_max
		break
	end
	group = struct('j', j, 't_j', t_j, 'trajlets', {{}}, 'fragments', {{}});
	for i = 1:length(fit_batch.fits)
		if ~fit_batch.fits{i}.empty  
			k_1 = j - (fit_batch.fits{i}.n_off - n_off_0);
			k_2 = k_1 + N_steps - 1;
			if k_1 >= 1 && k_2 <= length(fit_batch.fits{i}.fit.T_opt)
				trajlet = struct(...
					'X', fit_batch.fits{i}.fit.X_opt(k_1:k_2, :), ...
					'V', fit_batch.fits{i}.fit.V_opt(k_1:k_2, :), ...
					'U', fit_batch.fits{i}.fit.U_opt(k_1:k_2, :), ...
					'type', fit_batch.fits{i}.type, ...
					...'v_des', fit_batch.fits{i}.v_des
					'vmag_des', fit_batch.fits{i}.vmag_des, ...
					'vxabs_des', fit_batch.fits{i}.vxabs_des);
				group.trajlets = [group.trajlets, trajlet];
			elseif k_2 >= 1 && k_1 <= length(fit_batch.fits{i}.fit.T_opt)
				k_1_old = k_1;
				k_1 = max([1, k_1]);
				k_2 = min([length(fit_batch.fits{i}.fit.T_opt), k_2]);
				fragment = struct(...
					'X', fit_batch.fits{i}.fit.X_opt(k_1:k_2, :), ...
					'V', fit_batch.fits{i}.fit.V_opt(k_1:k_2, :), ...
					'U', fit_batch.fits{i}.fit.U_opt(k_1:k_2, :), ...
					'type', fit_batch.fits{i}.type, ...
					'k_off', k_1 - k_1_old, ...
					...'v_des', fit_batch.fits{i}.v_des
					'vmag_des', fit_batch.fits{i}.vmag_des, ...
					'vxabs_des', fit_batch.fits{i}.vxabs_des);
				group.fragments = [group.fragments, fragment];
			end
		end
	end
	seq = [seq, group];
	j = j + N_steps;
end