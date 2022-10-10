function slBusOut = localisation(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    slBusOut.Timestamp = double(msgIn.Timestamp);
    slBusOut.PosA = single(msgIn.PosA);
    slBusOut.PosB = single(msgIn.PosB);
    slBusOut.RotA = single(msgIn.RotA);
    slBusOut.RotB = single(msgIn.RotB);
end
