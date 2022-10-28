function samples = selectsamples(ij, position_condition, N_steps)
if length(ij) == 2
	s1 = sprintf('s_p00001/fit_batches_vdes/fit_batch_%i_%i.mat', ij(1), ij(2));	
else
	s1 = sprintf('s_p00001/fit_batches_vdes/fit_batch_%i.mat', ij);
end
fit_batch = vario(s1, 'fit_batch');

h = 0.05;
t_1 = fit_batch.window(1);
t_2 = fit_batch.window(2);
[seq, t_0] = mktrajletgroupsequence(fit_batch, N_steps, h, t_1, t_2);

samples = {};
for i = 1:length(seq)
	agg = aggregatetrajlets(seq{i}, N_steps);
	% keep only those trajlets which are defined at all times
	jj = all(agg.def, 1);
	jj2 = repelem(jj, 2);
	Xdef = agg.X(:, jj2);
	Vdef = agg.V(:, jj2);
	Adef = agg.A(:, jj2);
	Udef = agg.U(:, jj2);
	type_def = agg.type(jj);
	v_des_def = agg.v_des(jj);
	% keep only those trajlets satisfying the position condition
	jj_ = position_condition(Xdef);
	jj2_ = repelem(jj_, 2);
	if ~isempty(jj_)
		states = [Xdef(:, jj2_), Vdef(:, jj2_), Adef(:, jj2_)];
		samples = [samples, struct(...
			's', states(1, :), ...
			'states', states(2:end, :), ...
			'u', Udef(2:end, jj2_), ...
			'v_des', v_des_def(jj_), ...
			'type', type_def(jj_) ...
		)];
	end
end