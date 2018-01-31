classdef ObjectDetector < handle
    %OBJECTDETECTOR Summary of this class goes here
    %   Detailed explanation goes here
        
    %
    % Abstract methods 
    %
    methods(Abstract)
        
        % Find object in pointcloud and return x,y,z in pointcloud
        pose = findObject(obj,pointCloud)        
    end
   

end
