function slBusOut = IMU(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    slBusOut.Roll = single(msgIn.Roll);
    slBusOut.Pitch = single(msgIn.Pitch);
    slBusOut.Yaw = single(msgIn.Yaw);
    slBusOut.Accelx = single(msgIn.Accelx);
    slBusOut.Accely = single(msgIn.Accely);
    slBusOut.Accelz = single(msgIn.Accelz);
    slBusOut.Posx = single(msgIn.Posx);
    slBusOut.Posy = single(msgIn.Posy);
    slBusOut.Timestamp = double(msgIn.Timestamp);
end
