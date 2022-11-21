
classdef vehicles < ros.Message
    %vehicles MATLAB implementation of utils/vehicles
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'utils/vehicles' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'de73bb5b781774cb3107d20c52b302aa' % The MD5 Checksum of the message definition
        PropertyList = { 'ID' 'Timestamp' 'PosA' 'PosB' 'RotA' 'RotB' } % List of non-constant message properties
        ROSPropertyList = { 'ID' 'timestamp' 'posA' 'posB' 'rotA' 'rotB' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ID
        Timestamp
        PosA
        PosB
        RotA
        RotB
    end
    methods
        function set.ID(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'vehicles', 'ID');
            obj.ID = uint8(val);
        end
        function set.Timestamp(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'vehicles', 'Timestamp');
            obj.Timestamp = single(val);
        end
        function set.PosA(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'vehicles', 'PosA');
            obj.PosA = single(val);
        end
        function set.PosB(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'vehicles', 'PosB');
            obj.PosB = single(val);
        end
        function set.RotA(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'vehicles', 'RotA');
            obj.RotA = single(val);
        end
        function set.RotB(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'vehicles', 'RotB');
            obj.RotB = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.utils.vehicles.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.utils.vehicles(strObj);
        end
    end
end