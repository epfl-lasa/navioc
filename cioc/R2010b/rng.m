% Workaround for R2009b.
function rng(s)

rand('seed',s);
randn('seed',s);
