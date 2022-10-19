% Run timing tests on robot arm with fixed "flower petal" reward.
function robotarmtiming(a,s,r,test_file_name)
% 4 algorithm, 7 tests, 8 restarts

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
        'example_restarts',4,...
        'test_restarts',4,...
        'example_optimal',0,...
        'example_recompute_optimal',0,...
        'test_optimal',0,...
        'cells_state',10,...
        'cells_action',5,...
        'verbosity',4);
restarts = 8;
transfers = 4;
world = 'robotarm';

% Prepare MDP parameters.
mdp_cat_name = 'Links';
mdp_param_names = {'2','3','4','5','6','7','8'};
mdp_params = {struct('seed',0)};

% Set up Cartesian features.
%if a == 2 || a == 4, % This line should be used if we are using direct & Hessian inversion variants with dynamic robot arm
if a == 2,
    %algorithm_params{2}.learn_ard = 0;
    mdp_params{1}.feature_type = 'cartesian';
end;

mdp_params = repmat(mdp_params,1,length(mdp_param_names));

% Prepare test parameters.
test_params = {setdefaulttestparams(test_params)};
test_params = repmat(test_params,1,length(mdp_param_names));
mdp_params{1}.links = 2;
mdp_params{1}.linklen = repmat(5.0/2,1,2);
mdp_params{1}.linkmass = repmat(100.0,1,2);
mdp_params{2}.links = 3;
mdp_params{2}.linklen = repmat(5.0/3,1,3);
mdp_params{2}.linkmass = repmat(100.0,1,3);
mdp_params{3}.links = 4;
mdp_params{3}.linklen = repmat(5.0/4,1,4);
mdp_params{3}.linkmass = repmat(100.0,1,4);
mdp_params{4}.links = 5;
mdp_params{4}.linklen = repmat(5.0/5,1,5);
mdp_params{4}.linkmass = repmat(100.0,1,5);
mdp_params{5}.links = 6;
mdp_params{5}.linklen = repmat(5.0/6,1,6);
mdp_params{5}.linkmass = repmat(100.0,1,6);
mdp_params{6}.links = 7;
mdp_params{6}.linklen = repmat(5.0/7,1,7);
mdp_params{6}.linkmass = repmat(100.0,1,7);
mdp_params{7}.links = 8;
mdp_params{7}.linklen = repmat(5.0/8,1,8);
mdp_params{7}.linkmass = repmat(100.0,1,8);

% The dynamics robot arm uses Featherstone's algorithm to compute the real
% mass matrix. This is more realistic, but it also makes discretizations
% very difficult to deal with this. This makes prior methods (MaxEnt &
% OptV) perform very poorly, so we are not using it.
%{
% Switch to dynamics robot arm.
world = 'robotarmdyn';
for i=1:length(mdp_params),
    mdp_params{i}.linkmass = mdp_params{i}.linkmass*0.0005;
    % Choose whether we are using the nonlinear variant.
    if a == 3 || a == 4,
        mdp_params{i}.nonlinear = 1;
    end;
end;
%}
  
% Set random seeds.
for step=1:length(mdp_params),
    mdp_params{step}.seed = mdp_params{step}.seed+r-1;
end;

% Early out if this test is known to be infeasible.
%if a == 6 && s > 1, % Use this line if using dynamics robot arm.
if a == 3 && s > 1,
    return; % Can't do MaxEnt with more than 3 links (10^6 * 25 samples)
end;
%if a == 7 && s > 3, % Use this line if using dynamics robot arm.
if a == 4 && s > 3,
    return; % Can't do OptV with more than 5 links (5^4 samples)
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
%saveresults('Timing_Robot',test_params,test_metric_names,...
%    mdp_params,mdp_cat_name,mdp_param_names,algorithms,names,colors,order,...
%    restarts,series_result,transfer_result);
