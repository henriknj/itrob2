classdef KeyopCommunicator < handle
    %TurtleBotCommunicator - Class for communicating with Turtlebot
    %   TBOT = TurtleBotCommunicator()creates an object that allows for easy
    %   control and interaction with the Turtlebot platform. The
    %   constructor takes no arguments and subscribes to additional topics
    %   when the 'enable' functions are used
    %
    %   TurtleBotCommunicator methods:
    %       enableSound        - Enable sound publisher
    %       disableSound       - Disable sound publisher
    %       enableOdom         - Enable odometry subscriber
    %       disableOdom        - Disable odometry subscriber
    %       enableLaser        - Enable laser scan subscriber
    %       disableLaser       - Disable laser scan subscriber
    %       enableCamera       - Enable camera image subscriber
    %       disableCamera      - Disable camera image subscriber
    %       setPlot            - Turn on Kinect image display figure
    %       clearPlot          - Turn off Kinect image display figure
    %       plotTarget         - Add the target point to Kinect image figure
    %       checkHardware      - Determine if robot is hardware or simulation
    %       setSound           - Command the robot to make a sound
    %       resetOdom          - Reset the odometry topic
    %       setVelocity        - Set and publish velocity commands to Turtlebot
    %
    %   TurtleBotCommunicator properties:
    %       LaserData          - Data from laser scan, occupied points
    %       Pose               - Pose of TurtleBot
    %       ImColor            - Color image from Kinect
    %       BumpArray          - Array of bumper feedback
    %       CliffFlag          - Flag for cliff sensor
    %
    %   See also TurtleBotTeleoperationExample, TurtleBotObjectTrackingExample
    
    %   Copyright 2014 The MathWorks, Inc.
    properties (Access = public)
        LaserData = [];                 % Raw data from laser scan
        Pose = [];                      % Pose (x,y,t) data from /odom
        ImColor = [];                   % Color image from Kinect
        BumpArray = [];                 % Vector of bumper depression points
        CliffFlag = 0;                  % Flag for cliff sensor
        
    end
    
    properties (Access = protected)
        VelPub = [];                    % ROS publisher for velocity
        SoundPub = [];                  % ROS publisher for sound
        OdomResetPub = [];              % ROS publisher for odometry reset
        LaserSub = [];                  % ROS subscriber for laser scan
        OdomSub = [];                   % ROS subscriber for odometry
        ImageSub = [];                  % ROS subscriber for images
        BumpSub = [];                   % ROS subscriber for bumper
        CliffSub = [];                  % ROS subscriber for cliff sensor
        VelMsg = [];                    % ROS message for velocities
        SoundMsg = [];                  % ROS message for sounds
        OdomResetMsg = [];              % ROS message for odometry reset
        
        FigureHandle = [];              % Figure handle for Kinect image
        AxesHandle = [];                % Axes handle for Kinect image
        ImageOn = false;                % Flag for image display
        IsHardware = false;             % Enable if hardware present.
    end
    
    methods (Access = public)
        function h = KeyopCommunicator()
            %TurtleBotCommunicator Constructor
            
            % Set publishers
            h.VelPub = rospublisher('/mobile_base/commands/velocity');
            h.OdomResetPub = rospublisher('/mobile_base/commands/reset_odometry');
            
            % Set subscribers
            h.BumpSub = rossubscriber('/mobile_base/events/bumper', 'BufferSize', 5);
            h.BumpSub.NewMessageFcn = @h.bumpCallback;
            h.CliffSub = rossubscriber('/mobile_base/events/cliff', 'BufferSize', 5);
            h.CliffSub.NewMessageFcn = @h.cliffCallback;
            
            % Create messages
            h.VelMsg = rosmessage(rostype.geometry_msgs_Twist);
            h.OdomResetMsg = rosmessage(rostype.std_msgs_Empty);
            
        end
        
        function enableSound(h)
            %ENABLESOUND - Create publisher for sound
            
            try
                h.SoundPub = rospublisher('/mobile_base/commands/sound');
                disp('Enabled Sound Publisher');
            catch
                h.SoundPub = rospublisher('/mobile_base/commands/sound',rostype.kobuki_msgs_Sound);
                disp('No "/mobile_base/commands/sound" topic exists: Generating a new one');
            end
            h.SoundMsg = rosmessage(rostype.kobuki_msgs_Sound);
        end
        
        function disableSound(h)
            %DISABLESOUND - Removes publisher for sound topic
            
            if isempty(h.SoundPub)
                disp('Sound not enabled: No action taken');
            else
                h.SoundPub = [];
                h.SoundMsg = [];
                disp('Disabled Sound Publisher');
            end
        end
        
        function enableOdom(h)
            %ENABLEODOM - Subscribes to the odometry topic and indicates
            %   callback function
            
            h.OdomSub = rossubscriber('/odom', 'BufferSize', 25);
            receive(h.OdomSub,3);
            disp('Successfully Enabled Odometry');
            h.OdomSub.NewMessageFcn = @h.odomCallback;
        end
        
        function disableOdom(h)
            %DISABLEODOM - Unsubscribes from odometry topic
            
            if isempty(h.OdomSub)
                disp('Odometry not enabled: No action taken');
            else
                h.OdomSub = [];
                disp('Disabled Odometry Subscriber');
            end
        end
        
        function enableLaser(h)
            %ENABLELASER - Subscribes to the laser scan topic and
            % indicates callback function
            
            h.LaserSub = rossubscriber('/scan', 'BufferSize', 5);
            scan = receive(h.LaserSub,3);
            h.LaserData = readCartesian(scan);
            disp('Successfully Enabled Laser');
            h.LaserSub.NewMessageFcn = @h.laserCallback;
        end
        
        function disableLaser(h)
            %DISABLELASER - Unsubscribes from laser scan topic
            
            if isempty(h.LaserSub)
                disp('Laser not enabled: No action taken');
            else
                h.LaserSub = [];
                disp('Disabled Laser Scan Subscriber');
            end
        end
        
        function enableCamera(h)
            %ENABLECAMERA - Subscribes to an appropriate camera topic and
            % indicates callback
            
            h.IsHardware = exampleHelperTurtleBotIsPhysicalRobot(); % Determine if hardware exists
            if h.IsHardware
                h.ImageSub = rossubscriber('/camera/rgb/image_color/compressed', 'BufferSize', 5);
                img = receive(h.ImageSub,3);
                h.ImColor = readImage(img);
                disp('Successfully Enabled Camera (compressed image)');
            else
                h.ImageSub = rossubscriber('/camera/rgb/image_raw', 'BufferSize', 5);
                img = receive(h.ImageSub,3);
                h.ImColor = readImage(img);
                disp('Successfully Enabled Camera (raw image)');
            end
            h.ImageSub.NewMessageFcn = @h.imageColorCallback;
        end
        
        function disableCamera(h)
            %DISABLECAMERA - Unsubscribes from camera topic
            
            if isempty(h.ImageSub)
                disp('Image not enabled: No action taken');
            else
                h.ImageSub = [];
                disp('Disabled Image Subscriber');
            end
        end
        
        function setPlot(h)
            %SETPLOT - Initializes image plotting
            
            h.FigureHandle = figure('Name','Turtlebot Camera','CloseRequestFcn',@h.closeFigure);
            h.AxesHandle = axes('Parent',h.FigureHandle);
            h.ImageOn = true;
        end
        
        function clearPlot(h)
            %CLEARPLOT - Disables image plotting
            
            h.FigureHandle = [];
            h.AxesHandle = [];
            h.ImageOn = false;
        end
        
        function plotTarget(h, c)
            %PLOTTARGET - Plots the target point on the image
            
            if h.ImageOn
                hold(h.AxesHandle,'on');
                plot(h.AxesHandle,c(1),c(2),'+','Color','r','MarkerSize',30);
            end
        end
        
        function hardware = checkHardware(h)
            %CHECKHARDWARE - Checks if Turtlebot is real or simulated
            
            hardware = (h.IsHardware == 1);
        end
        
        function setSound(h, type)
            %SETSOUND - Plays a sound from the Turtlebot
            
            if strcmp('turn on', type)
                type = 0;
            elseif strcmp('turn off', type)
                type = 1;
            elseif strcmp('recharge start', type)
                type = 2;
            elseif strcmp('press button', type)
                type = 3;
            elseif strcmp('error sound', type)
                type = 4;
            elseif strcmp('start cleaning', type)
                type = 5;
            elseif strcmp('cleaning end', type)
                type = 6;
            end
            if any(type == [0 1 2 3 4 5 6])
                h.SoundMsg.Value = type;
                send(h.SoundPub,h.SoundMsg);
            else
                disp('Invalid Sound');
            end
        end
        
        function resetOdom(h)
            %RESETODOM - Resets the odometry for the Turtlebot
            
            send(h.OdomResetPub,h.OdomResetMsg);
        end
        
        function setVelocity(h, vLin, vAng)
            %SETVELOCITY(vLin,vAng) - Set linear and angular velocities
            % for the Turtlebot
            
            h.VelMsg.Linear.X = vLin;
            h.VelMsg.Angular.Z = vAng;
            send(h.VelPub,h.VelMsg);
        end
    end
    
    methods (Access = protected)
        
        function delete(h)
            %DELETE - Private method to delete the object
            
            % Kill subscribers
            disp('Shutting down Turtlebot subscribers');
            disableOdom(h);
            disableLaser(h);
            disableCamera(h);
            drawnow;
        end
        
        function laserCallback(h, ~, message)
            %LASERCALLBACK - Collect laser scan data and fill it into a
            %   class variable
            
            h.LaserData = readCartesian(message) * [0 1; -1 0];
        end
        
        function odomCallback(h, ~, message)
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
                h.Pose = [xpos, ypos, theta];
            end
            
            count = count+1;
        end
        
        function imageColorCallback(h, ~, message)
            %IMAGECALLBACK - Retrieve a color image and fill it into a
            % class variable
            
            persistent count;
            if isempty(count)
                count = 0;
            end
            
            % Slow the image rate by a factor of 5
            if mod(count,5) == 0
                h.ImColor =  readImage(message);
                
                if h.ImageOn && (mod(count,20))
                    image(h.ImColor,'Parent',h.AxesHandle);
                end
            end
            
            count = count+1;
        end
        
        function bumpCallback(h, ~, message)
            %BUMPCALLBACK - Return an array of bumper contact as a class
            % variable
            
            h.BumpArray = message.Data;
        end
        
        function cliffCallback(h, ~, ~)
            %CLIFFCALLBACK - Raise a flag when cliff sensor is activated
            
            h.CliffFlag = 1;
        end
        
        function closeFigure(h,~,~)
            %CLOSEFIGURE - Callback function that deletes the figure
            % handle
            delete(h.FigureHandle);
            clearPlot(h);
        end
        
    end
    
end

