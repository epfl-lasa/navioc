% Sample some random controls in the Highway environment.
function u = highwaysamplecontrols(n,mdp_data)

u = [rand(n,1)*1.0 randn(n,1)*2.0];
