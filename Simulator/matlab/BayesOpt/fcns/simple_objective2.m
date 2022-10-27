function y = simple_objective2(x)
%SIMPLE_MULTIOBJECTIVE is a simple multi-objective fitness function.
%
% The multi-objective genetic algorithm solver assumes the fitness function
% will take one input x where x is a row vector with as many elements as
% number of variables in the problem. The fitness function computes the
% value of each objective function and returns the vector value in its one
% return argument y.

%   Copyright 2007 The MathWorks, Inc. 
% y(2) = (x-2)^2 + 20;
simple_fun = @(x) (x-2)^2 + 20;
y = table2array(rowfun(simple_fun,x));
end



