% Fill in default parameters for the OptV algorithm.
function algorithm_params = optvdefaultparams(algorithm_params)

% Create default parameters.
default_params = struct(...
    'bases',32,...
    'basis_iters',50,...
    'seed',0);

% Set parameters.
algorithm_params = filldefaultparams(algorithm_params,default_params);
