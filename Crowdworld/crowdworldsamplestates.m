function s = crowdworldsamplestates(n, mdp_data)

if strcmp(mdp_data.state_sampling_method, 'corridor')
	poslim = [mdp_data.half_width, mdp_data.half_height];
	poslimrep = repmat(poslim, [1, mdp_data.n_ped]);
	pos = (2*rand(n, mdp_data.udims) - 1).*poslimrep;
	s = [pos, mdp_data.v_des, zeros(1, mdp_data.udims)];
elseif strcmp(mdp_data.state_sampling_method, 'pair')
	pos = -6*mdp_data.v_des + randn*[1, 0, -1, 0] + randn*[0, 1, 0, -1]/50;
	s = [pos, mdp_data.v_des, zeros(1, mdp_data.udims)];
else % 'random'
	poslim = [mdp_data.half_width, mdp_data.half_height];
	vellim = [mdp_data.v_max, mdp_data.v_max];
	acclim = [mdp_data.accel_max, mdp_data.accel_max];
	poslimrep = repmat(poslim, [1, mdp_data.n_ped]);
	vellimrep = repmat(vellim, [1, mdp_data.n_ped]);
	acclimrep = repmat(acclim, [1, mdp_data.n_ped]);

	limrep = [poslimrep, vellimrep, acclimrep];

	base = 2*rand(n, mdp_data.dims);
	s = (base - 1.0).*limrep;
end