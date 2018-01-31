%
% MarkerLocator
%
classdef MarkerLocator < handle
    %MARKERLOCATOR MarkerLocator locates a marker in a given area.
    %   Detailed explanation goes here
    
    properties
        robot;
        detector;
        controller;
        depthSub;
        depthMsg;
        
        % Offset to marker in meters
        markerOffset = 0.4 + 0.1725 + 0.4;
        
        % Robot camera offset
        cameraOffset = 0.1;
        
        % Walls
        wall1 = [];
        wall2 = [deg2rad(110),deg2rad(70)];
        wall3 = [];
    end
    
    methods(Access = public)
        
        %
        % MarkerLocator
        %
        function obj = MarkerLocator(detector, controller, offset)
            
            if(~isa(detector,'ObjectDetector'))
                msgID = 'MarkerLocator:BadDetectorArgument';
                msg = 'Argument not of type ObjecDetector';
                throw(MException(msgID,msg));                
            end
            
            obj.robot = TurtleBot.getInstance();
            obj.detector = detector;
            obj.controller = controller;
   
            % Offset to marker
            obj.markerOffset = offset;
            
        end
        
        %
        % Locate the position of the object
        %
        function position = getMarkerPosition(obj, startpoint)
            position = [];
            objectFound = false;
            wallindexCount = 1;
            moveToStart = false;
            
            % turn
            obj.controller.moveToOrientation(deg2rad(90));
            
            % find the object
            while(~objectFound)
                pause(2);
                
                % get pointcloud
                pcd = obj.robot.getPointCloud();
                               
                % locate object in pointcloud            
                locatedObject = obj.detector.findObject(pcd);

                % did we find the object or not?
                if(~locatedObject)
                    if moveToStart
                        obj.controller.moveToPoint(startpoint);
                        wallindexCount = 1;
                        moveToStart = false;
                    end
                    
                    if wallindexCount <= size(obj.wall2,2)
                        % change direction
                        obj.controller.moveToOrientation(obj.wall2(wallindexCount));

                        % increment job
                        wallindexCount = wallindexCount + 1;

                    else
                        obj.controller.moveToOrientation(deg2rad(-10));
                        obj.controller.moveToPoint([20 2.4]);
                        obj.controller.moveToOrientation(deg2rad(0));
                        
                        moveToStart = true;
                    end                             
                else
                    
                    % find the objectposition on map
                    [x,y] = obj.calculateMarkerPosition(locatedObject)

                    position = [x y];

                    % found
                    objectFound = true;
                    
                    % check if position on map is valid
                    %if(~SMap.getInstance().isPointOccupied([x y]))
                    %    objectFound = true;
                    %    position(1) = x;
                    %    position(2) = y;
                    %else
                        % what to do here?
                    %end
                    
                end
                
            end
            
        end
        
        % Find the object position in map
        function [x,y] = calculateMarkerPosition(obj,position)
        
            % currentpose
            curPose = obj.controller.getCurrentPose();
            
            % Rotate pose to fit map coordinates
            X = position(3) - obj.markerOffset;
            Y = -position(1); 
            robotTheta = curPose(3);
            X_r = X*cos(robotTheta) - Y*sin(robotTheta);
            Y_r = X*sin(robotTheta) + Y*cos(robotTheta);
            
            x = X_r + curPose(1);
            y = Y_r + curPose(2);
            
        end
        
    end
    
end

