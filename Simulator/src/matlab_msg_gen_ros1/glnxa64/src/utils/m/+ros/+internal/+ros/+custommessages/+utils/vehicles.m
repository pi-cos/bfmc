function [data, info] = vehicles
%vehicles gives an empty data for utils/vehicles
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'utils/vehicles';
[data.ID, info.ID] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Timestamp, info.Timestamp] = ros.internal.ros.messages.ros.default_type('single',1);
[data.PosA, info.PosA] = ros.internal.ros.messages.ros.default_type('single',1);
[data.PosB, info.PosB] = ros.internal.ros.messages.ros.default_type('single',1);
[data.RotA, info.RotA] = ros.internal.ros.messages.ros.default_type('single',1);
[data.RotB, info.RotB] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'utils/vehicles';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'ID';
info.MatPath{2} = 'timestamp';
info.MatPath{3} = 'posA';
info.MatPath{4} = 'posB';
info.MatPath{5} = 'rotA';
info.MatPath{6} = 'rotB';
