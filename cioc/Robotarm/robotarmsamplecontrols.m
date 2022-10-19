% Sample some random controls for robot arm.
function u = robotarmsamplecontrols(n,mdp_data)

u = randn(n,mdp_data.udims)*5.0;
