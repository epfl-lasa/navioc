function rand_samples_data = resamplerand(samples_data, n_particles, u_std)

orig_state = warning;
warning('off','all');
rmpath Crowdworld
warning(orig_state);
addpath Crowdworld2

n_samples = length(samples_data.samples);
n_rand_samples = n_particles*n_samples;
k = 0;
rand_samples_data = struct(...
	'samples', {cell(1, n_rand_samples)}, ...
	'mdp_data_arr', {cell(1, n_rand_samples)} ...
);
for i = 1:n_samples
	x = samples_data.samples{i}.s;
	%U = randn([size(samples_data.samples{i}.u), n_particles]);
	for j = 1:n_particles
		k = k + 1;
		%u = samples_data.samples{i}.u + 
		u = randn(size(samples_data.samples{i}.u))*u_std;
		[states, A, B] = crowdworldcontrol(samples_data.mdp_data_arr{i}, x, u);
		rand_samples_data.samples{k} = struct('s', x, 'u', u, 'states', states);
		rand_samples_data.mdp_data_arr{k} = samples_data.mdp_data_arr{i};
	end
end
