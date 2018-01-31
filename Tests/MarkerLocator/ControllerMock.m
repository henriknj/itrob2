classdef ControllerMock < MockObject
    %CONTROLLERMOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ReturnSuccess
        startPose;
    end
    
    methods
        function obj = ControllerMock(startPose)
            obj.ReturnSuccess = true;
            obj.startPose = startPose;
        end
        
        function result = getCurrentPose(obj)
            obj.addToCallStack({'getCurrentPose'});
            result = [0,0,0];
        end
    end
    
end

