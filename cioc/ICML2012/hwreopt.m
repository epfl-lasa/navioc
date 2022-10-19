
% Instructions:
% This script should be run on the output from highwayrun in order to
% reoptimize the test set. Initially, the test set is initialized to the
% same controls as the examples, which artificially "helps" it be similar.
% To avoid this bias, this script restarts all test trajectories at random,
% so they are completely unaware of the holdout user examples. This only
% reoptimizes the test examples, not the training examples.

for TEST_NUM=1:length(test_result.test_samples),
    % First, reoptimize test from a base placeholder reward to get it to move.
    fprintf(1,'Running preliminary optimization with AME reward.\n');
    mdp = 'highway';
    mdp_data = test_result.mdp_data;
    mdp_data.optimizes = 1;
    [~,u,~,~] = optimizetrajectory(...
        test_result.irl_result.test_samples{TEST_NUM}.s,...
        size(test_result.irl_result.test_samples{TEST_NUM}.u,1),...
        mdp_data,[],'highway',test_result.irl_result.ame_result.reward,test_result.reward,0,5);

    % Now optimize with respect to the true reward function.
    fprintf(1,'Running final optimization.\n');
    test_result.irl_result.test_samples{TEST_NUM}.u = u;
    proxyu = u;

    mdp_data = rmfield(mdp_data,'optimizes');
    [~,test_result.irl_result.test_samples{TEST_NUM}.u,~] = reoptimizetrajectory(...
        test_result.irl_result.test_samples{TEST_NUM},...
        mdp_data,'highway',test_result.irl_result.reward,...
        test_result.reward,1);

end;
