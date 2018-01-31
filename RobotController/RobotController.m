classdef RobotController < handle
    %ROBOTCONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(SetAccess = public)
        % Dt Time step of the robot
        Dt = 0.10;
        
        % RobotRadius Physical radius of the robot
        RobotRadius = 0.17;
        
        % GoalRadius Determines the distance the robot has to be within the 
        % goal to reach it
        GoalRadius = 0.05;
        
        % PurePursuit parameters
        DesiredLinearVelocity = 0.3;
        MaxAngularVelocity = 3;
        LookaheadDistance = 0.5;
        
        % Weights for Scan and Odometry
        OdomWeight = 0.2;
        ScanWeight = 0.3;
		
        % Rotation parameters
        AngularVelocityOffset = 0.25;
        
		% Laser scan parameters
		CalEvery_x_LaserScan = 10;
		poseCorrectEvery_x_CalLaserScan = 5;
		
    end
    
    properties(SetAccess = private)
        % CurrentPose Current pose of the robot
        CurrentPose;
        StartPose;
        Vel;
        AngVel;
        
        % Map The map of the environment
        Map;
        
        % IsReady Is the robot ready for commands?
        FollowingTrajectory;
        
        % PurePursuit controller
        PursuitController;
        
        % Turtlebot
        Robot;
        
        % Plot handles
        TrajPlot = [];
        PosePlot = [];
        HeadingPlot = [];
    end
    
    methods(Access = public)
        function obj = RobotController(startPose)
            % Setup startpose and map
            obj.CurrentPose = startPose;
            obj.StartPose = startPose;
            obj.Map = SMap.getInstance().getOccupancyGrid();
            
            % Set IsReady
            obj.FollowingTrajectory = false;
            
            % Robot instance
            obj.Robot = TurtleBot.getInstance();
            obj.Robot.resetOdom();
            
        end
        
        function moveToPoint(obj, point)
            %moveToPoint
            %   moveToPoint(obj,point) moves the robot along a trajectory
            %   to the specified point.
            points = [obj.CurrentPose(1:2);
                      point];
            trajectory = obj.makeTrajectory(points);
            obj.moveAlongTrajectory(trajectory);
        end
        
        function moveToOrientation(obj,orientation)
            odomOrientation = obj.getOdomOrientation();
            obj.rotate(orientation, odomOrientation);
        end
        
        function pose = getCurrentPose(obj)
            %getCurrenPose
            %   pose = getCurrentPose(obj) returns the current pose of the
            %   robot
           pose = obj.CurrentPose;
        end
        
        function moveAlongTrajectory(obj, traj)
            %moveAlongTrajectory
            %   moveAlongTrajectory(obj,traj) moves the robot along a given
            %   trajectory.
            if obj.FollowingTrajectory
                return;
            end
            obj.FollowingTrajectory = true;
            
            % Setup PurePursuit controller from object parameters
            obj.PursuitController = PurePursuit;
            obj.PursuitController.Waypoints = traj;
            obj.PursuitController.DesiredLinearVelocity = obj.DesiredLinearVelocity;
            obj.PursuitController.MaxAngularVelocity = obj.MaxAngularVelocity;
            obj.PursuitController.LookaheadDistance = obj.LookaheadDistance;

            % Start timer for sending velocity commands
            t = obj.createTimer(obj.Dt);
            start(t);
            
            % Sleep until done
            while obj.FollowingTrajectory
                pause(2);
            end
        end
        
        function trajectory = makeTrajectory(obj, points)
            %makeTrajectory
            %   trajectory = makeTrajectory(obj, point) creates a
            %   trajectory between current pose and the given point.
            start = obj.CurrentPose(1:2);
            
            % Create trajectory
            % Takes via-points, max velocities, empty, start-point,
            % time-step and acceleration
            trajectory = mstraj(points, [obj.DesiredLinearVelocity obj.DesiredLinearVelocity], [], start, obj.Dt, 1);
            
            global verbose
            if verbose
                obj.drawTrajectory(trajectory);
            end
        end
        
        %
        % Functions for kinematics
        %
        function t = createTimer(obj, period)
            t = timer;
            t.UserData = obj;
            t.StartFcn = @RobotController.timerStart;
            t.TimerFcn = @RobotController.timerTimeout;
            t.StopFcn = @RobotController.timerCleanup;
            t.Period = period;
            t.ExecutionMode = 'fixedSpacing';
        end
        
        function timerStart(mTimer, evt)
            obj = mTimer.UserData;
            
            obj.setNextVelocity();
            obj.startStopwatch();
        end
        
        function timerTimeout(mTimer, evt)
            obj = mTimer.UserData;
            
            % Calculate current pose
            timeElapsed = obj.timeElapsed();
            obj.CurrentPose = obj.estimatePose(obj.CurrentPose, timeElapsed, obj.Vel, obj.AngVel);
            
            % Calculate distance to goal
            robotCurrentLocation = obj.CurrentPose(1:2);
            robotGoal = obj.PursuitController.Waypoints(end,:);
            distanceToGoal = norm(robotCurrentLocation - robotGoal);

            if distanceToGoal > obj.GoalRadius && obj.FollowingTrajectory
                obj.setNextVelocity();
            else
                stop(mTimer);
                obj.FollowingTrajectory = false;
            end
            
            % Draw robot on map if verbose
            global verbose;
            if verbose
                obj.drawRobot();
            end
        end
        
        function timerCleanup(mTimer, evt)
            obj = mTimer.UserData;
            obj.stopStopwatch();
            delete(mTimer);
        end
        
        function setNextVelocity(obj)
            % Compute the controller outputs, i.e., the inputs to the robot
            [vel, angvel] = step(obj.PursuitController, obj.CurrentPose);

            % Drive the robot using the controller outputs.
            obj.Robot.setVelocity(vel, angvel);
            
            obj.Vel = vel;
            obj.AngVel = angvel;
        end
        
        function newPose = estimatePose(obj, oldPose, timeElapsed, v, w)
            % Calculate new pose
            x = oldPose(1) + v*cos(oldPose(3))*timeElapsed;
            y = oldPose(2) + v*sin(oldPose(3))*timeElapsed;
            theta = obj.normalize(oldPose(3) + w*timeElapsed);
            newPose = [x, y, theta];
        end
        
        %
        % Stopwatch functions
        %
        function time = timeElapsed(obj)
            time = toc;
            tic;
        end
        
        function startStopwatch(obj)
            tic;
        end
        
        function stopStopwatch(obj)
            toc;
        end
        
        %
        % Message callbacks
        %
        function odomCallback(obj, ~, message)
            %ODOMCALLBACK - Collect odometry data and fill it into a class
            % variable
            if ~obj.FollowingTrajectory
                return;
            end
            
            persistent count
            if isempty(count)
                count = 0;
            end
            
            if ~mod(count,10)
                % Estimate current pose
                timeElapsed = obj.timeElapsed();
                obj.CurrentPose = obj.estimatePose(obj.CurrentPose, timeElapsed, obj.Vel, obj.AngVel);
                
                % Get odometry pose
                pose = message.Pose.Pose;
                quat = pose.Orientation;
                angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
                theta = angles(1);
                
                % Rotate pose to fit map coordinates
                X = pose.Position.X;
                Y = pose.Position.Y;
                robotTheta = obj.StartPose(3);
                X_r = X*cos(robotTheta) - Y*sin(robotTheta);
                Y_r = X*sin(robotTheta) + Y*cos(robotTheta);
                odomPose = [X_r, Y_r, theta] + obj.StartPose;
                odomPose(3) = obj.normalize(odomPose(3));
                
                % Calculate offset
                correctionOffset = odomPose - obj.CurrentPose;
                correctionOffset(3) = obj.normalize(correctionOffset(3))
                
                % Update estimate
                obj.CurrentPose = obj.CurrentPose + [2*obj.OdomWeight*correctionOffset(1:2) obj.OdomWeight*correctionOffset(3)];
                obj.CurrentPose(3) = obj.normalize(obj.CurrentPose(3));
            end
            
            count = count+1;
        end
        
        function laserCallback(obj,~, message)
            global verbose
            if ~obj.FollowingTrajectory
                return;
            end

            persistent count
            if isempty(count)
                count = 0;
            end
			
            persistent laserDataNo
            if isempty(laserDataNo)
                laserDataNo = 1;
            end			
			
            persistent laserOffset
            if isempty(laserOffset)
				laserOffset = zeros(obj.poseCorrectEvery_x_CalLaserScan,3);
            end		

            persistent laserOffset2
            if isempty(laserOffset2)
				laserOffset2 = zeros(3);
            end	
            
            if ~mod(count,obj.CalEvery_x_LaserScan)
                % Estimate current pose
                timeElapsed = obj.timeElapsed();
                obj.CurrentPose = obj.estimatePose(obj.CurrentPose, timeElapsed, obj.Vel, obj.AngVel);

                % Extract lines from laser scan
                lines = lineExtraction(message, false, false);
				% Get laser pose offset
                laserOffset(laserDataNo,:) = laserCorrectPose(lines, obj.CurrentPose, obj.Map);

                if laserDataNo == obj.poseCorrectEvery_x_CalLaserScan
					laserOffset2 = nanmedian(laserOffset,1)
					% change NaN values to 0
					laserOffset2(isnan(laserOffset2)) = 0;
                    % Update estimate
                    obj.CurrentPose(3) = obj.CurrentPose(3) + obj.ScanWeight*laserOffset2(3);
                    
					laserDataNo = 1;
				else
					laserDataNo = laserDataNo+1;
                end

                obj.CurrentPose(1:2) = obj.CurrentPose(1:2) + obj.ScanWeight*0.2*(1/obj.CalEvery_x_LaserScan)*laserOffset2(1:2);
            end
            
            count = count+1;
        end
        
        function cliffCallback(obj,~, message)
            obj.FollowingTrajectory = false;
            rosshutdown;clear;
        end
        
        function bumpCallback(obj,~, message)
            obj.FollowingTrajectory = false;
            rosshutdown;clear;
        end 
    end
    
    methods(Access = private)
        function drawRobot(obj)
            % Draw robot every 10 times it is called
            persistent count
            if isempty(count)
                count = 0;
            end

            % Delete plot handles and redraw
            if ~mod(count,10)
                if ~isempty(obj.PosePlot)
                    delete(obj.PosePlot);
                    delete(obj.HeadingPlot);
                end
                [obj.PosePlot, obj.HeadingPlot ] = ...
                    drawRobotToMap([obj.CurrentPose(1:2) rad2deg(obj.CurrentPose(3))]);
            end

            count = count+1;
        end
        
        function drawTrajectory(obj, trajectory)
            figure(99);
            hold on;
            if ~isempty(obj.TrajPlot)
                delete(obj.TrajPlot);
            end
            obj.TrajPlot = plot(trajectory(:,1), trajectory(:,2))
        end
        
        function orientation = getOdomOrientation(obj)
            odomPose = obj.Robot.getOdom();
            quat = odomPose.Orientation;
            angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
            orientation = angles(1);
        end
        
        function angle = normalize(obj, angle)
            angle = atan2(sin(angle),cos(angle));
        end
        
        function rotate(obj, RotateTo, OdomOrientation)
            global verbose
            
            % Constants
            dt = obj.Dt;
            angularVel = obj.MaxAngularVelocity/3;
            angularVelOffset = obj.AngularVelocityOffset;
            
            % Current orientation
            pose = obj.CurrentPose;
            currentOrientation = pose(3);
            odomAngularOffset = currentOrientation-OdomOrientation;
            
            % Desired rotation angle
            goalOrientation = RotateTo;
            rotationAngle = obj.normalize(goalOrientation-currentOrientation);

            % Get odometry pose
            % theta_goal determines when the rotate algorithm stops
            theta = OdomOrientation
            theta_goal = theta+rotationAngle

            % Calculate error
            previous_error = 0;
            integral = 0;
            error = obj.normalize(theta_goal - theta);
            
            % PID-parameters
            Kp = 1;
            Ki = 0.009;
            Kd = 0.1;

            figure(99), hold on;
            drawRobotToMap([pose(1:2) rad2deg(pose(3))]);

            % While the error is more than 0.2 degrees, keep rotating
            hasRotated = false;
            while abs(error) > deg2rad(0.2)
                hasRotated = true;
                
                % Get integral and derivative
                integral = error * dt + integral;
                derivative = (error - previous_error) / dt;

                % PID-controlled output velocity
                output_vel = ...
                    Kp * error * angularVel + ...
                    Ki * integral + ...
                    Kd * derivative + ... 
                    sign(error) * angularVel * angularVelOffset;
                
                % Make sure output_vel does not escalate due to i/d
                if abs(output_vel) > angularVel
                    output_vel = sign(error) * angularVel
                end
                
                % Set velocity for dt
                obj.Robot.setVelocity(0,output_vel);
                pause(dt);

                % Get new pose from odometry
                theta = obj.getOdomOrientation();%obj.normalize(theta + output_vel * dt);%
                pose = [pose(1:2) theta];
                
                % Calculate new error
                previous_error = error;
                error = obj.normalize(theta_goal - theta)

                % Draw the robot if verbose is on
                if verbose
                    if ~isempty(obj.PosePlot)
                        delete(obj.PosePlot);
                        delete(obj.HeadingPlot);
                    end
                    [obj.PosePlot, obj.HeadingPlot ] = ...
                        drawRobotToMap([pose(1:2) rad2deg(obj.normalize(pose(3)+odomAngularOffset))]);
                end
            end
            
            % Update the current pose accordingly, if the robot has rotated
            if hasRotated
                obj.CurrentPose = [pose(1:2) obj.normalize(pose(3)+odomAngularOffset)];
                pose(3)+odomAngularOffset
                obj.CurrentPose
            end
            
            obj.Robot.setVelocity(0,0);
        end
    end
    
end

