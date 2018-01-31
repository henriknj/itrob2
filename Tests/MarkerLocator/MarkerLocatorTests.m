classdef MarkerLocatorTests < matlab.unittest.TestCase
    %MARKERLOCATORTESTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        locator;
    end
    
    methods(TestMethodSetup)
        function setUpMarker(obj)
            % Create instance
            global verbose;
            verbose = true;
            
            startPose = [0,0,0];
            detector = CircleObjectDetector(6,12,'dark',0.9);
            controller = ControllerMock(startPose);
            obj.locator = MarkerLocator(detector,controller,0.40);
        end
    end
 
    methods(TestMethodTeardown)
    end
    
    methods(Test)  

        %
        % Marker located on wall1
        %
        function testCase_locatedOnWall1(obj)
            
            load('Data\componentlab\inside_1meterFront.mat');
            pcd = cloud;  
            
            % Where do we belive it to be
            realPosition = [0,0,0];
            
            % Find object in pointcloud
            %objectPosition = obj.detector.findObject(pcd);
            
            % We should add a tolerance
            obj.locator.calculateMarkerPosition([1 1 1]);

           
        end
    end
    
end

