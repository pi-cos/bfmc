

restoredefaultpath; clear all; close all; clc;

%%


% fun = @simple_objective;
vars = optimizableVariable('xvar',[-10 +10],'Type','real');
fun = @simple_objective;

% x = 1;
fun(array2table(1))

%%

results = bayesopt(fun,vars,'IsObjectiveDeterministic',true);