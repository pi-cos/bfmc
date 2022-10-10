function [data, info] = environmental
%environmental gives an empty data for utils/environmental
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'utils/environmental';
[data.ObstacleId, info.ObstacleId] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'utils/environmental';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'obstacle_id';
info.MatPath{2} = 'x';
info.MatPath{3} = 'y';
