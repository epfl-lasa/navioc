% Sample some random states in the Objectworld.
function s = objectworldsamplestates(n,mdp_data)

s = bsxfun(@times,rand(n,2),mdp_data.bounds) * mdp_data.sensor_basis;
