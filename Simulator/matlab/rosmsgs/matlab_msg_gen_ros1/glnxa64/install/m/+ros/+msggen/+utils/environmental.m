
classdef environmental < ros.Message
    %environmental MATLAB implementation of utils/environmental
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'utils/environmental' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'a1acf3f1b0fd75ef5b1b19cde1c2ce7f' % The MD5 Checksum of the message definition
        PropertyList = { 'ObstacleId' 'X' 'Y' } % List of non-constant message properties
        ROSPropertyList = { 'obstacle_id' 'x' 'y' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ObstacleId
        X
        Y
    end
    methods
        function set.ObstacleId(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'environmental', 'ObstacleId');
            obj.ObstacleId = uint8(val);
        end
        function set.X(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'environmental', 'X');
            obj.X = single(val);
        end
        function set.Y(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'environmental', 'Y');
            obj.Y = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.utils.environmental.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.utils.environmental(strObj);
        end
    end
end
