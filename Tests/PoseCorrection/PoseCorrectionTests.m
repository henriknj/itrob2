classdef PoseCorrectionTests < matlab.unittest.TestCase
    %CIRCLEDETECTIONTESTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        scansub;
    end
    
    methods(TestMethodSetup)
        function setUpScanner(obj)
            RosNodeInit('192.168.1.100','192.168.1.101');
            obj.scansub = rossubscriber('/scan');
        end
    end
 
    methods(TestMethodTeardown)
    end
    
    methods(Test)  

        %
        % Waypoint1 is the first location in the componentlab
        %
        function testCase_wayPointOne(obj)
            
            scandata = receive(obj.scansub);  
            
            % Where do we belive it to be
            realPose = [12.36 18.64 -90];
            
            % Find object in pointcloud
            presumedPose = [12.8, 18.64, -90];
            
            % Extract lines and correct pose
            lines = lineExtraction(scandata, true, false);
            correctedPose = posePrediction(lines, presumedPose, Map.getInstance().getOccupancyGrid());
            
            realPose
            correctedPose
            
            % We should add a tolerance
            assertEqual(correctedPose,realPose);
            
        end
    end
    
end

