classdef TurtleBotTmpTest < handle
    %TURTLEBOTTMPTEST Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot;
        OdomPose;
    end
    
    methods
        function obj = TurtleBotTmpTest()
           obj.robot = TurtleBot.getInstance(); 
        end
        
        function start(obj)
           obj.robot.connect('192.168.1.100','192.168.1.101');
           %obj.robot.enableOdom(obj);
           %obj.robot.enableCamera();
           obj.robot.enableLaser(obj);
        end
        
        function stop(obj)
           obj.robot.delete(); 
        end
        
        function pcd = pointcloud(obj)
           pcd = obj.robot.getPointCloud(); 
        end
        
        function laserCallback(obj, ~, message)
            showdetails(message);
            obj.robot.disableLaser(obj);
            
        end
        
        function odomCallback(obj, ~, message)
            %ODOMCALLBACK - Collect odometry data and fill it into a class
            % variable
            
            persistent count
            if isempty(count)
                count = 0;
            end
            
            if ~mod(count,10)
                % Extract the x, y, and theta coordinates every 10 calls
                pose = message.Pose.Pose;
                xpos = pose.Position.X;
                ypos = pose.Position.Y;
                quat = pose.Orientation;
                angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
                theta = angles(1);
                obj.OdomPose = [xpos, ypos, theta];
                disp(obj.OdomPose);
                obj.robot.disableOdom();
                showdetails(message);
                disp(GetTimeStamp(message));
            end
            
            count = count+1;
        end
        
    end
    
end

