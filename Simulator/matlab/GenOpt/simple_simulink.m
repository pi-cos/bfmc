function y = simple_simulink(x)
%SIMPLE_MULTIOBJECTIVE is a simple simulink dependent fitness function.
%
% The multi-objective genetic algorithm solver assumes the fitness function
% will take one input x where x is a row vector with as many elements as
% number of variables in the problem. The fitness function computes the
% value of each objective function and returns the vector value in its one
% return argument y.

%   Copyright 2007 The MathWorks, Inc. 
% assignin('base','sim_in_x','x');
set_param('test_genOpt/Constant2','Value',num2str(x))
sim_res = sim('test_genOpt.slx');
y = [sim_res.y1(end);sim_res.y2(end)];


