% Sample some random controls in the Objectworld.
function u = objectworldsamplecontrols(n,mdp_data)

u = randn(n,mdp_data.udims)*5.0;
