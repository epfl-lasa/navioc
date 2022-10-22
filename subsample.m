function result = subsample(sample, mdp_data, features_pt, n_agents)
%SUBSAMPLE Summary of this function goes here
%   Detailed explanation goes here

idx = randperm(mdp_data.n_ped, n_agents);

jj = reshape([idx*2-1; idx*2], [1, n_agents*2]);
jj_all = [jj, jj + 2*mdp_data.n_ped, jj + 4*mdp_data.n_ped];

mdp_data.n_ped = n_agents;
mdp_data.v_des = mdp_data.v_des(jj);
mdp_data.dims = 6*n_agents;
mdp_data.udims = 2*n_agents;

features_pt{2}.x_des = mdp_data.v_des;
features_pt{3}.x_des = zeros(size(mdp_data.v_des));

sample.s = sample.s(jj_all);
sample.states = sample.states(:, jj_all);
sample.u = sample.u(:, jj);

result = struct('sample', sample, 'mdp_data', mdp_data, 'features_pt', {features_pt}, 'idx', idx);
end

