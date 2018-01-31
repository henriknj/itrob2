classdef CircleDetectionTests < matlab.unittest.TestCase
    %CIRCLEDETECTIONTESTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        detector;
        magic_constant = 10;
    end
    
    methods(TestMethodSetup)
        function setUpDetector(obj)
            % Create instance
            global verbose;
            verbose = true;
            obj.detector = CircleObjectDetector(4,8,'dark',0.9);     
            warning('off','all');
        end
    end
 
    methods(TestMethodTeardown)
        function closeDetector(obj)
            close all;
        end
    end
    
    methods(Test)  

        %
        % Wall1 doorcenter
        %
        function testcase_wall1_door(obj)
            
            load('Data\componentlab\wall1\waypoint1_markerLeftBot.mat');
            objectPosition = obj.detector.findObject(pcd,'2d');
            
            pause(2);
            
            %
            % Object around [x,y] = 208,267
            %
            withXInArea = (208 + obj.magic_constant) > objectPosition(1) && ... 
                (208-obj.magic_constant) < objectPosition(1) ;
            withYinArea = (267 + obj.magic_constant) > objectPosition(2) && ...
                (267-obj.magic_constant) < objectPosition(2);
            
            obj.assertEqual(double(withXInArea),1);
            obj.assertEqual(double(withYinArea),1);

        end
        
        %
        % Marker Bottom
        %
        function wall2_leftRightAngle80_markerBot(obj)
            
            load('Data\componentlab\wall2\waypoint1_rightAngle_80_markerBot.mat');
            
            %
            % Lowest sensitivity
            % - image is 30,31 in diameter
            %
            obj.detector.setRadius(5,30);
            obj.detector.setSensitivity(0.75);
            
            % Find object
            objectPosition = obj.detector.findObject(pcd,'2d');
            
            pause(2);
            
            %
            % Object around [x,y] = 520,326
            %
            withXInArea = (520 + obj.magic_constant) > objectPosition(1) && ... 
                (520-obj.magic_constant) < objectPosition(1) ;
            withYinArea = (326 + obj.magic_constant) > objectPosition(2) && ...
                (326-obj.magic_constant) < objectPosition(2);
            
            obj.assertEqual(double(withXInArea),1);
            obj.assertEqual(double(withYinArea),1);
                 
        end
        
        %
        % Marker Center
        %
        function wall2_leftRightAngle80_markerCenter(obj)
            
            load('Data\componentlab\wall2\waypoint1_rightAngle_80_markerCenter.mat');
            
            %
            % Lowest sensitivity
            % - image is 28,29 in diameter
            %
            obj.detector.setRadius(5,30);
            obj.detector.setSensitivity(0.75);
            
            % Find object
            objectPosition = obj.detector.findObject(pcd,'2d');
            
            pause(2);
            
            %
            % Object around [x,y] = 431,194
            %
            withXInArea = (431 + obj.magic_constant) > objectPosition(1) && ... 
                (431-obj.magic_constant) < objectPosition(1) ;
            withYinArea = (194 + obj.magic_constant) > objectPosition(2) && ...
                (194-obj.magic_constant) < objectPosition(2);
            
            obj.assertEqual(double(withXInArea),1);
            obj.assertEqual(double(withYinArea),1);
                     
        end        
        %
        % Marker Top
        %
        function wall2_leftRightAngle80_markerTop(obj)
            
            load('Data\componentlab\wall2\waypoint1_rightAngle_80_markerTop.mat');
            
            %
            % Lowest sensitivity
            % - image is 28,29 in diameter
            %
            obj.detector.setRadius(5,30);
            obj.detector.setSensitivity(0.80);
            
            % Find object
            objectPosition = obj.detector.findObject(pcd,'2d');
            
            pause(2);
            
            %
            % Object around [x,y] = 418,64
            %
            withXInArea = (418 + obj.magic_constant) > objectPosition(1) && ... 
                (418-obj.magic_constant) < objectPosition(1) ;
            withYinArea = (64 + obj.magic_constant) > objectPosition(2) && ...
                (64-obj.magic_constant) < objectPosition(2);
            
            obj.assertEqual(double(withXInArea),1);
            obj.assertEqual(double(withYinArea),1);
                     
        end
        
        %
        % Marker Left Bottom
        %
        function wall2_leftAngle100_markerBot(obj)
            
            load('Data\componentlab\wall2\waypoint1_leftAngle_100_markerBot.mat');
            
            %
            % Lowest sensitivity
            % - image is 29,30 in diameter
            %
            obj.detector.setRadius(5,30);
            obj.detector.setSensitivity(0.75);
            
            % Find object
            objectPosition = obj.detector.findObject(pcd,'2d');
            
            pause(2);
            
            %
            % Object around [x,y] = 184,326
            %
            withXInArea = (184 + obj.magic_constant) > objectPosition(1) && ... 
                (184-obj.magic_constant) < objectPosition(1) ;
            withYinArea = (326 + obj.magic_constant) > objectPosition(2) && ...
                (326-obj.magic_constant) < objectPosition(2);
            
            obj.assertEqual(double(withXInArea),1);
            obj.assertEqual(double(withYinArea),1);
                     
        end          

        %
        % Marker Left Center
        %
        function wall2_leftAngle100_markerCenter(obj)
            
            load('Data\componentlab\wall2\waypoint1_leftAngle_100_markerCenter.mat');
            
            %
            % Lowest sensitivity
            % - image is 34,35 in diameter
            %
            obj.detector.setRadius(5,30);
            obj.detector.setSensitivity(0.77);
            
            % Find object
            objectPosition = obj.detector.findObject(pcd,'2d');
            
            pause(2);
            
            %
            % Object around [x,y] = 364,166
            %
            withXInArea = (364 + obj.magic_constant) > objectPosition(1) && ... 
                (364-obj.magic_constant) < objectPosition(1) ;
            withYinArea = (166 + obj.magic_constant) > objectPosition(2) && ...
                (166-obj.magic_constant) < objectPosition(2);
            
            obj.assertEqual(double(withXInArea),1);
            obj.assertEqual(double(withYinArea),1);
                     
        end
        
        %
        % Marker Left Top
        %
        function wall2_leftAngle100_markerTop(obj)
            
            load('Data\componentlab\wall2\waypoint1_leftAngle_100_markerTopInvisible.mat');
            
            %
            % Lowest sensitivity
            % - image is 31,32 in diameter
            %
            obj.detector.setRadius(5,30);
            obj.detector.setSensitivity(0.85);
            
            % Find object
            objectPosition = obj.detector.findObject(pcd,'2d');
            
            pause(2);
            
            %
            % Object around [x,y] = 325,23
            %
            withXInArea = (325 + obj.magic_constant) > objectPosition(1) && ... 
                (325-obj.magic_constant) < objectPosition(1) ;
            withYinArea = (23 + obj.magic_constant) > objectPosition(2) && ...
                (23-obj.magic_constant) < objectPosition(2);
            
            obj.assertEqual(double(withXInArea),1);
            obj.assertEqual(double(withYinArea),1);
                     
        end           
        
    end
    
end

