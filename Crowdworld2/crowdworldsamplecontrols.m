function u = crowdworldsamplecontrols(n, mdp_data)

u = randn(n, mdp_data.udims)*mdp_data.u_sigma;