%% 
restoredefaultpath; clear all; close all; clc;

%%

open('test_genOpt.slx')
simple_simulink(3)

%%

FitnessFunction = @simple_simulink;
numberOfVariables = 1;
[x,fval] = gamultiobj(FitnessFunction,numberOfVariables)

save simulink_test_results

%%

FitnessFunction = @simple_multiobjective;
numberOfVariables = 1;
[x,fval] = gamultiobj(FitnessFunction,numberOfVariables)

save multiobjective_test_results

%%

clearvars

load simulink_test_results
fval_sim = fval;
figure(1)
grid on
hold on
plot(fval(:,1), fval(:,2),'*')

clearvars
load multiobjective_test_results
fval_mo = fval;
figure(1)
grid on
hold on
plot(fval(:,1), fval(:,2),'*')

%%

return
%% constrained

%     min F(x) = [objective1(x); objective2(x)] 
%      x
%      
%     subject to  -1.5 <= x <= 0 (bound constraints)
%
%     where, objective1(x) = (x+2)^2 - 10, and
%            objective2(x) = (x-2)^2 + 20

% |gamultiobj| accepts linear inequality constraints in the form |A*x <= b|
% and linear equality constraints in the form |Aeq*x = beq| and bound
% constraints in the form |lb <= x <= ub|. We pass |A| and |Aeq| as
% matrices and |b|, |beq|, |lb|, and |ub| as vectors. Since we have no
% linear constraints in this example, we pass |[]| for those inputs.

A = []; b = [];
Aeq = []; beq = [];
lb = -1.5;
ub = 0;
x = gamultiobj(FitnessFunction,numberOfVariables,A,b,Aeq,beq,lb,ub);

%% visualization

options = optimoptions(@gamultiobj,'PlotFcn',{@gaplotpareto,@gaplotscorediversity});
gamultiobj(FitnessFunction,numberOfVariables,[],[],[],[],lb,ub,options);