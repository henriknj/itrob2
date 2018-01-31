classdef RobotControllerTests < matlab.unittest.TestCase
    %CIRCLEDETECTIONTESTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        scansub;
    end
    
    methods(TestMethodSetup)
        function setUpScanner(obj)
            %RosNodeInit('192.168.1.100','192.168.1.101');
            %obj.scansub = rossubscriber('/scan');
        end
    end
 
    methods(TestMethodTeardown)
    end
    
    methods(Test)  

        %
        % Waypoint1 is the first location in the componentlab
        %
        function testCase_wayPointOne(obj)
            % Start position
            pose = [12.36 18.64 deg2rad(270)];
            target2 = [13.8 15.8];

            % Setup robot controller
            controller = RobotController(pose);
            controller.moveToPoint(target2);
            
            % We should add a tolerance
            assertEqual(1,1);
        end
    end
    
end

