function [data, info] = iMU
%IMU gives an empty data for utils/IMU
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'utils/IMU';
[data.Roll, info.Roll] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Pitch, info.Pitch] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Yaw, info.Yaw] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Accelx, info.Accelx] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Accely, info.Accely] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Accelz, info.Accelz] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Posx, info.Posx] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Posy, info.Posy] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Timestamp, info.Timestamp] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'utils/IMU';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'roll';
info.MatPath{2} = 'pitch';
info.MatPath{3} = 'yaw';
info.MatPath{4} = 'accelx';
info.MatPath{5} = 'accely';
info.MatPath{6} = 'accelz';
info.MatPath{7} = 'posx';
info.MatPath{8} = 'posy';
info.MatPath{9} = 'timestamp';
