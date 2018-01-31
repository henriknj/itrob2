%
% Turtlebot singleton to communicate with ROS machine
%
classdef TurtleBot < handle
    %TURTLEBOT - Connect to turtlebot and enable sensors/actuators
    %   Detailed explanation goes here
    
    properties(Access=public)
        VelPub = [];                    % ROS publisher for velocity
        SoundPub = [];                  % ROS publisher for sound
        OdomResetPub = [];              % ROS publisher for odometry reset
        OdomPub = [];                   % ROS publisher for odometry pose
        LaserSub = [];                  % ROS subscriber for laser scan
        OdomSub = [];                   % ROS subscriber for odometry
        ImageSub = [];                  % ROS subscriber for images
        DepthSub = [];                  % ROS subscriber for depthimage
        BumpSub = [];                   % ROS subscriber for bumper
        CliffSub = [];                  % ROS subscriber for cliff sensor
        VelMsg = [];                    % ROS message for velocities
        SoundMsg = [];                  % ROS message for sounds
        OdomResetMsg = [];              % ROS message for odometry reset
        ConnectedToMaster = false;      % Boolean to indicate status
        Timeout = 10;
    end
    
    properties(Access=public)
        LaserData = [];                 % Raw data from laser scan
        Pose = [];                      % Pose (x,y,t) data from /odom
    end
    
    methods (Access=private)
        function obj = TurtleBot()
       
        end       
        
        function setupPubSub(obj) 
            
            if(obj.ConnectedToMaster) 
                % Set publishers
                obj.VelPub = rospublisher('/mobile_base/commands/velocity');
                obj.OdomResetPub = rospublisher('/mobile_base/commands/reset_odometry');
                obj.OdomPub = rospublisher('/mobile_base/commands/reset_odometry');

                % Create messages
                obj.VelMsg = rosmessage(rostype.geometry_msgs_Twist);
                obj.OdomResetMsg = rosmessage(rostype.std_msgs_Empty);       
            end
        end
        
        function delete(h)
            %DELETE - Private method to delete the object
            
            % Kill subscribers
            disp('Shutting down Turtlebot subscribers');
            disableOdom(h);
            disableLaser(h);
            disableCamera(h);
        end
    end
    
    methods (Static)
       function obj = getInstance()
            persistent uniqueInstance
            if isempty(uniqueInstance)
                obj = TurtleBot();
                uniqueInstance = obj;
            else
                obj = uniqueInstance;
            end
        end 
    end
    
    methods(Access=public)
        
        function connect(obj,master,localIp)
            % CONNECT - Connect to ROS master process
            try 
                % Connect to ROS master
                rosuri = strcat('http://', master, ':11311');

                setenv('ROS_MASTER_URI', rosuri);
                setenv('ROS_IP',localIp);
                rosinit(rosuri,'NodeHost',localIp);
                
                obj.ConnectedToMaster = true;
                obj.setupPubSub();
                
            catch e
                switch e.identifier
                    case 'ROSException'
                    case 'robotics:ros:node:GlobalNodeRunningError'
                        disp('Already connected to master node');
                    otherwise
                        rethrow(e);     
                end
            end
            
        end
        
        function enableBumper(obj,callbackObj)
            % ENABLEBUMPER - Disable the bumper sensor
            %
            obj.BumpSub = rossubscriber('/mobile_base/events/bumper', 'BufferSize', 5);
            obj.BumpSub.NewMessageFcn = @callbackObj.bumpCallback;
        end
        
        function disableBumper(obj)
            % DISABLEBUMPER - Disable the bumper sensor
            %
            if isempty(obj.BumpSub)
                disp('Bumper not enabled: No action taken');
            else
                obj.BumpSub = [];
                disp('Disabled bumper Subscriber');
            end
        end
        
        function enableCliff(obj,callbackObj)
            % ENABLECLIFF - Enable the cliff sensor
            %
            obj.CliffSub = rossubscriber('/mobile_base/events/cliff', 'BufferSize', 5);
            obj.CliffSub.NewMessageFcn = @callbackObj.cliffCallback;
        end
        
        function disableCliff(obj)
            % DISABLECLIFF - Disable the cliff sensor
            %
            if isempty(obj.CliffSub)
                disp('Bumper not enabled: No action taken');
            else
                obj.CliffSub = [];
                disp('Disabled bumper Subscriber');
            end
        end
        
        function enableCamera(obj)
            % ENABLECAMERA - Enable the camera
            %
            obj.DepthSub = rossubscriber('/camera/depth_registered/points', 'BufferSize', 5);
            receive(obj.DepthSub,obj.Timeout);
            disp('Enabled camera');     
        end
        
        function disableCamera(obj)
            % DISABLECAMERA - Disable the camera
            %
            if isempty(obj.DepthSub)
                disp('Camera not enabled: No action taken');
            else
                obj.DepthSub = [];
                disp('Disabled Camera Subscriber');
            end            
        end
        
        function enableOdom(obj, callbackObj)
            %ENABLEODOM - Subscribes to the odometry topic and indicates
            %   callback function
            obj.OdomSub = rossubscriber('/robot_pose_ekf/odom_combined');
            receive(obj.OdomSub,obj.Timeout);
            disp('Successfully Enabled Odometry');

            % add callback to list of callbacks
            obj.OdomSub.NewMessageFcn = @callbackObj.odomCallback;
        end
        
        function odom = getOdom(obj)
            %GETODOM - Polls and returns current Odom
            temp = receive(obj.OdomSub,obj.Timeout);
            odom = temp.Pose.Pose;
        end
        
        function disableOdom(obj)
            %DISABLEODOM - Unsubscribes from odometry topic
            if isempty(obj.OdomSub)
                disp('Odometry not enabled: No action taken');
            else
                obj.OdomSub = [];
                disp('Disabled Odometry Subscriber');
            end
        end
        
        function resetOdom(obj)
            %RESETODOM - Resets the odometry for the Turtlebot
            send(obj.OdomResetPub,obj.OdomResetMsg);
        end
        
        function setVelocity(obj, vLin, vAng)
            %SETVELOCITY(vLin,vAng) - Set linear and angular velocities
            obj.VelMsg.Linear.X = vLin;
            obj.VelMsg.Angular.Z = vAng;
            send(obj.VelPub,obj.VelMsg);                
        end
       
        function setSound(obj, type)
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
                if ~isempty(obj.SoundPub) 
                    obj.SoundMsg.Value = type;
                    send(obj.SoundPub,obj.SoundMsg);
                end
            else
                disp('Invalid Sound');
            end
        end
       
        function enableSound(obj)
            %ENABLESOUND - Create publisher for sound       
                try
                    obj.SoundPub = rospublisher('/mobile_base/commands/sound');
                    disp('Enabled Sound Publisher');
                catch
                    obj.SoundPub = rospublisher('/mobile_base/commands/sound',rostype.kobuki_msgs_Sound);
                    disp('No "/mobile_base/commands/sound" topic exists: Generating a new one');
                end
                obj.SoundMsg = rosmessage(rostype.kobuki_msgs_Sound);
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
        
        function enableLaser(obj, object)
            %ENABLELASER - Subscribes to the laser scan topic and
            % indicates callback function
            obj.LaserSub = rossubscriber('/scan');
            scan = receive(obj.LaserSub,obj.Timeout);
            obj.LaserData = readCartesian(scan);
            disp('Successfully Enabled Laser');
            obj.LaserSub.NewMessageFcn = @object.laserCallback;                
        
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
        
        function time = gettimeofday(~)
            % GETTIMEOFTDAY - Get the time on the ros machine
            %
            time = rostime('now');
        end
        
        function pcd = getPointCloud(obj)
            % GETPOINTCLOUD - Get a PointCloud2 type from camera
            if isempty(obj.DepthSub)
                disp('Camera not enabled: No action taken');
            else
                pcd = receive(obj.DepthSub,obj.Timeout);
            end   
        end
        
    end
    
end