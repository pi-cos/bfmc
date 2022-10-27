

restoredefaultpath; clear all; close all; clc;

%%

addpath('fcns')

%%


% fun = @simple_objective;
vars = optimizableVariable('xvar',[-10 +10],'Type','real');
ii = 0;
fun = str2func(['simple_multiobjective_0',num2str(ii)]);

% x = 1;
fun(array2table(1))

%%

pareto_res = [];

for ii = 0:9
    fun = str2func(['simple_multiobjective_0',num2str(ii)]);
    results = bayesopt(fun,vars,'IsObjectiveDeterministic',true);
    pareto_res = [pareto_res;ii,table2array(results.XAtMinObjective),results.MinObjective];
end

fun = str2func(['simple_multiobjective_10']);
results = bayesopt(fun,vars,'IsObjectiveDeterministic',true);
pareto_res = [pareto_res;ii,table2array(results.XAtMinObjective),results.MinObjective];

%%

% alpha = 0:0.1:1;
figure(1); 
clf
for ii = 1:11
    x = pareto_res(ii,2);
    grid on; hold on; plot(simple_objective1(array2table(x)), simple_objective2(array2table(x)),'*')
end
