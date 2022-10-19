% Sample some random states in the robot arm domain.
function s = robotarmsamplestates(n,mdp_data)

s = [(rand(n,mdp_data.links)*pi*2 - pi) zeros(n,mdp_data.links)];
