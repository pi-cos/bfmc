function [data, info] = localisation
%localisation gives an empty data for utils/localisation
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'utils/localisation';
[data.Timestamp, info.Timestamp] = ros.internal.ros.messages.ros.default_type('double',1);
[data.PosA, info.PosA] = ros.internal.ros.messages.ros.default_type('single',1);
[data.PosB, info.PosB] = ros.internal.ros.messages.ros.default_type('single',1);
[data.RotA, info.RotA] = ros.internal.ros.messages.ros.default_type('single',1);
[data.RotB, info.RotB] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'utils/localisation';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'timestamp';
info.MatPath{2} = 'posA';
info.MatPath{3} = 'posB';
info.MatPath{4} = 'rotA';
info.MatPath{5} = 'rotB';
