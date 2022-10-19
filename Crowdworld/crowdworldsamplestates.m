function s = crowdworldsamplestates(n, mdp_data)

poslim = [mdp_data.half_width, mdp_data.half_height];
vellim = [mdp_data.v_max, mdp_data.v_max];
acclim = [mdp_data.accel_max, mdp_data.accel_max];
poslimrep = repmat(poslim, [1, mdp_data.n_ped]);
vellimrep = repmat(vellim, [1, mdp_data.n_ped]);
acclimrep = repmat(acclim, [1, mdp_data.n_ped]);

limrep = [poslimrep, vellimrep, acclimrep];

base = 2*rand(n, mdp_data.dims);
s = (base - 1.0).*limrep;