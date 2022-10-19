% Run example tests on objectworld with fixed "flower petal" reward.
function objectworldtest(a,s,r,example_optimal,test_file_name)
% 4 algorithm, 5 tests, 8 restarts

% Set up algorithms.
%algorithms = {'ame','gpirl','lqr','maxent','optv'};
%algorithm_params = {struct(),struct(),struct(),struct(),struct()};
%names = {'Linear/Hessian','GP/Hessian','Linear/LQR','MaxEnt','OptV'};
%colors = {[0 0 0.5],[0 0 0],[0.5 0.5 0.5],[0.2 0.8 0.2],[0.8 0.2 0.2]};
%order = [1 2 3 4 5];
algorithms = {'ame','gpirl','maxent','optv'};
algorithm_params = {struct(),struct(),struct(),struct()};
names = {'Linear','Nonlinear','MaxEnt','OptV'};
colors = {[0 0 0.5],[0 0 0],[0.2 0.8 0.2],[0.8 0.2 0.2]};
order = [1 2 3 4];
disp(names);
fprintf(1,'Starting run %i %i %i\n',a,s,r);

% Set up constants.
test_metric_names = metricnames();
test_params = struct(...
        'training_sample_lengths',16,...
        'training_samples',8,...
        'test_samples',32,...
        'example_restarts',1,...
        'test_restarts',1,...
        'example_optimal',example_optimal,...
        'example_recompute_optimal',example_optimal,...
        'test_optimal',1,...
        'cells_state',30,...
        'cells_action',10,...
        'verbosity',4);
restarts = 8;
world = 'objectworld';

% Prepare MDP parameters.
mdp_cat_name = 'Examples';
mdp_param_names = {'4','8','16','32','64'};
mdp_params = {struct('sensors',2,'motors',2,'seed',0)};
mdp_params = repmat(mdp_params,1,length(mdp_param_names));

% Prepare test parameters.
test_params = {setdefaulttestparams(test_params)};
test_params = repmat(test_params,1,length(mdp_param_names));
test_params{1}.training_samples = 4;
test_params{2}.training_samples = 8;
test_params{3}.training_samples = 16;
test_params{4}.training_samples = 32;
test_params{5}.training_samples = 64;

% Set random seeds.
for step=1:length(mdp_params),
    mdp_params{step}.seed = mdp_params{step}.seed+r-1;
end;

% Run single test.
test_result = runtest(algorithms{a},algorithm_params{a},...
                      world,mdp_params{s},test_params{s});

% Save test result and auxiliary data.
save([test_file_name '_' num2str(a) '_' num2str(s) '_' num2str(r) '.mat'],...
    'test_file_name','test_params','test_metric_names',...
    'mdp_params','mdp_cat_name','mdp_param_names',...
    'algorithms','names','colors','order','restarts','test_result');

% Run tests.
%series_result = runtestseries(algorithms,algorithm_params,...
%    test_params,world,mdp_params,restarts);

% Run transfer tests.
%transfer_result = [];
%transfer_result = runtransferseries(algorithms,series_result,...
%    mdp_model,test_params,world,mdp_params,restarts,transfers);

% Print.
%printstats(1,test_params,test_metric_names,...
%    mdp_params,mdp_cat_name,mdp_param_names,algorithms,names,restarts,...
%    series_result,transfer_result);

% Save.
%saveresults(test_file_name,test_params,test_metric_names,...
%    mdp_params,mdp_cat_name,mdp_param_names,algorithms,names,colors,order,...
%    restarts,series_result,transfer_result);
