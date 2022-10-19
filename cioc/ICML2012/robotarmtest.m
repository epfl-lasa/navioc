% Run example tests on robot arm with fixed "flower petal" reward.
function robotarmtest(a,s,r,cartesian,test_file_name)
% 4 algorithm, 5 tests, 8 restarts

% Set up algorithms.
%algorithms = {'ame','gpirl','ame','gpirl','lqr','maxent','optv'};
%algorithm_params = {struct(),struct(),struct(),struct(),struct(),struct(),struct()};
%names = {'Linear/Hessian','GP/Hessian','Linear/Direct','GP/Direct','Linear/LQR','MaxEnt','OptV'};
%colors = {[0 0 0.5],[0 0 0],[0 0.75 0.75],[0 0.5 0.5],[0.5 0.5 0.5],[0.2 0.8 0.2],[0.8 0.2 0.2]};
%order = [1 2 3 4 5 6 7];
algorithms = {'ame','gpirl''maxent','optv'};
algorithm_params = {struct(),struct(),struct(),struct()};
names = {'Linear','Nonlinear','MaxEnt','OptV'};
colors = {[0 0 0.5],[0 0 0],[0.2 0.8 0.2],[0.8 0.2 0.2]};
order = [1 2 3 4];
disp(names);
fprintf(1,'Starting run %i %i %i\n',a,s,r);

% Set up constants.
test_metric_names = metricnames();
test_params = struct(...
        'training_sample_lengths',32,...
        'training_samples',8,...
        'test_samples',32,...
        'example_restarts',1,...
        'test_restarts',1,...
        'example_optimal',1,...
        'example_recompute_optimal',1,...
        'test_optimal',1,...
        'cells_state',10,...
        'cells_action',5,...
        'verbosity',4);
restarts = 8;
world = 'robotarm';

% Prepare MDP parameters.
mdp_cat_name = 'Examples';
mdp_param_names = {'4','8','16','32','64'};
mdp_params = {struct(...
    'seed',0,...
    'links',2,...
    'linklen',[2.5 2.5],...
    'linkmass',[100.0 100.0])};

% The dynamics robot arm uses Featherstone's algorithm to compute the real
% mass matrix. This is more realistic, but it also makes discretizations
% very difficult to deal with this. This makes prior methods (MaxEnt &
% OptV) perform very poorly, so we are not using it.
%{
% Switch to dynamics robot arm.
world = 'robotarmdyn';
mdp_params{1}.linkmass = mdp_params{1}.linkmass*0.0005;
test_params.example_optimal = 0;
test_params.example_recompute_optimal = 0;
test_params.test_optimal = 0;
test_params.test_restarts = 4;
test_params.example_restarts = 4;

% Choose whether we are using the nonlinear variant.
if a == 3 || a == 4,
    mdp_params{1}.nonlinear = 1;
end;
%}

% Set up Cartesian features.
if cartesian,
    %algorithm_params{2}.learn_ard = 0;
    mdp_params{1}.feature_type = 'cartesian';
end;

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
