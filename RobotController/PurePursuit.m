classdef PurePursuit < matlab.System
    %PUREPURSUIT Create a controller to follow a set of waypoints
    %   The pure pursuit controller is a geometric controller for following
    %   a path. Given a set of waypoints, the pure pursuit controller
    %   computes linear and angular velocity control inputs for a given
    %   pose of a differential drive robot.
    %
    %   PP = robotics.PUREPURSUIT returns a pure pursuit system object, PP,
    %   that computes linear and angular velocity inputs for a differential
    %   drive robot using the PurePursuit algorithm.
    %
    %   PP = robotics.PUREPURSUIT('PropertyName', PropertyValue, ...) returns
    %   a pure pursuit object, PP, with each specified property set to
    %   the specified value.
    %
    %   Step method syntax:
    %
    %   [V, W] = step(PP, POSE) finds the linear velocity, V, and the
    %   angular velocity, W, for a 3-by-1 input vector POSE using the pure
    %   pursuit algorithm. The POSE is the current position,
    %   [x y orientation] of the robot. The output velocities V and W can
    %   be applied to a real or simulated differential drive robot to drive
    %   it along the desired waypoint sequence.
    %
    %   PUREPURSUIT methods:
    %
    %   step        - Compute linear and angular velocity control commands
    %   release     - Allow property value changes
    %   reset       - Reset internal states to default
    %   clone       - Create pure pursuit object with same property values
    %   isLocked    - Locked status (logical)
    %   info        - Get additional information about the object
    %
    %   PUREPURSUIT properties:
    %
    %   Waypoints               - Waypoints representing a path to follow
    %   MaxAngularVelocity      - Desired maximum angular velocity
    %   LookaheadDistance       - Lookahead distance to compute controls
    %   DesiredLinearVelocity   - Desired constant linear velocity
    %
    %   Example
    %
    %       % Create a pure pursuit object
    %       pp = robotics.PurePursuit;
    %
    %       % Assign a sequence of waypoints
    %       pp.Waypoints = [0 0;1 1;3 4];
    %
    %       % Compute control inputs for initial pose [x y theta]
    %       [v, w] = step(pp, [0 0 0]);
    %
    %   See also robotics.BinaryOccupancyGrid, robotics.PRM.
    
    %   Copyright 2014-2015 The MathWorks, Inc.
    %
    %   References:
    %
    %   [1] J. M. Snider, "Automatic Steering Methods for Autonomous
    %       Automobile Path Tracking", Robotics Institute, Carnegie Mellon
    %       University, Pittsburgh, PA, USA, Tech. Report CMU-RI-TR-09-08,
    %       Feb. 2009.
    %#codegen
    %#ok<*EMCA>
    
    properties (Nontunable)
        %Waypoints The waypoints representing a path to follow
        %
        %   Default: []
        Waypoints
    end
    
    properties
        %MaxAngularVelocity Maximum angular velocity in radians per second
        %   The controller saturates the absolute angular velocity output
        %   at MaxAngularVelocity value.
        %
        %   Default: 1.0
        MaxAngularVelocity      = 1.0
        
        %LookaheadDistance Lookahead distance in meters
        %   The lookahead distance changes the response of the controller.
        %   Higher lookahead distance produces smooth paths but the robot
        %   takes larger turns at corners. Smaller lookahead distance
        %   closely follows the path and robot takes sharp turns but may
        %   produce oscillations in the path.
        %
        %   Default: 1.0
        LookaheadDistance       = 1.0
        
        %DesiredLinearVelocity Desired linear velocity in meters per second
        %   The controller assumes that the robot drives at a constant
        %   linear velocity and the computed angular velocity is
        %   independent of the linear velocity.
        %
        %   Default: 0.1
        DesiredLinearVelocity   = 0.1
    end
    
    properties (Access = private)
        %Path X-Y coordinates of finely discretized waypoints
        Path
        
        %ProjPointIdx Index of a point on path closest to the current pose
        ProjPointIdx
        
        %DeltaDistance Distance between discretized waypoints
        DeltaDistance
        
        %LookaheadPoint Coordinates of the lookahead point
        LookaheadPoint
        
        %LastPose Last pose used to compute robot velocity commands
        LastPose
    end
    
    methods
        function set.Waypoints(obj, waypts)
            %set.Waypoints Setter for Waypoints property
            
            validateattributes(waypts, {'double'},{'finite', 'real','ncols',2}, ...
                'PurePursuit', 'Waypoints');
            
            isInvalidNumOfWaypoints = size(waypts,1) < 2;
            coder.internal.errorIf(isInvalidNumOfWaypoints, ...
                'robotics:robotalgs:purepursuit:NeedTwoWaypoints', 'waypoints');
            obj.Waypoints = waypts;
        end
        
        function set.MaxAngularVelocity(obj, omega)
            %set.MaxAngularVelocity Setter for Waypoints property
            
            validateattributes(omega, {'double'},{'nonnan', 'real', 'scalar', ...
                'positive', 'nonempty'}, 'PurePursuit', 'MaxAngularVelocity');
            obj.MaxAngularVelocity = omega;
        end
        
        function set.LookaheadDistance(obj, dist)
            %set.LookaheadDistance Setter for LookaheadDistance property
            
            validateattributes(dist, {'double'},{'nonnan', 'real', 'scalar', ...
                'positive', 'nonempty'},'PurePursuit', 'LookaheadDistance');
            obj.LookaheadDistance = dist;
        end
        
        function set.DesiredLinearVelocity(obj, val)
            %set.DesiredLinearVelocity Setter for DesiredLinearVelocity
            
            validateattributes(val, {'double'},{'nonnan', 'real', 'scalar', ...
                'positive', 'nonempty'},'PurePursuit', 'DesiredLinearVelocity');
            obj.DesiredLinearVelocity = val;
        end
        
        function obj = PurePursuit(varargin)
            %PurePursuit Constructor
            setProperties(obj,nargin,varargin{:},'Waypoints', ...
                'DesiredLinearVelocity', 'MaxAngularVelocity', ...
                'LookaheadDistance');
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj)
            %setupImpl Construct discretized waypoints
            
            coder.internal.errorIf(isempty(obj.Waypoints), ...
                'robotics:robotalgs:purepursuit:EmptyWaypoints')
            
            % Get discretized waypoints
            [xd, yd, delta] = ...
                robotics.algs.internal.equidistantPoints(obj.Waypoints);
            obj.Path = [xd(:) yd(:)];
            obj.DeltaDistance = delta;
            obj.ProjPointIdx = 0;
            obj.LookaheadPoint = zeros(1,2);
            obj.LastPose = zeros(1,3);
        end
        
        function [v, w] = stepImpl(obj,curPose)
            %stepImpl Compute control commands
            
            validateattributes(curPose, {'double'}, {'nonnan', 'real','vector', 'numel',3}, ...
                'step', 'pose');
            
            %Update the last pose
            obj.LastPose = [curPose(1) curPose(2) curPose(3)];
            
            % Compute lookahead index increment
            lookAheadIdxDist = round(obj.LookaheadDistance/obj.DeltaDistance);
            
            % First time, check all points and find the nearest point
            % as the projection
            if obj.ProjPointIdx == 0
                obj.ProjPointIdx = findNearest(obj, curPose);
            else
                % Search for nearest point only in few next
                % points
                updateProj(obj, curPose, lookAheadIdxDist);
            end
            
            % Compute carrot point based on projection. If near end, then
            % the end point is the carrot point
            lookAheadIdx = obj.ProjPointIdx + lookAheadIdxDist;
            if (lookAheadIdx < size(obj.Path, 1))
                obj.LookaheadPoint = obj.Path(lookAheadIdx, :);
            else
                obj.LookaheadPoint = obj.Path(end, :);
            end
            
            % Angle between robot heading and the line connecting robot and
            % the carrot point
            slope = atan2((obj.LookaheadPoint(2) - curPose(2)), ...
                (obj.LookaheadPoint(1) - curPose(1)));
            alpha = robotics.internal.angdiff(curPose(3), slope);
            
            % Angular velocity command for a differential drive robot is
            % equal to the desired curvature to be followed by the robot.
            
            % Using eq. (2) on page 11 of Reference [1].
            w = (2*sin(alpha))/obj.LookaheadDistance;
            
            % Pick a constant rotation when robot is facing in the opposite
            % direction of the path
            if abs(abs(alpha) - pi) < 1e-12
                w = sign(w)*1;
            end
            
            w = obj.saturate(w);
            
            % Only drive forward when angular velocity is low!
            %if w > 0.05
            %    v = 0;
            %else
                v = obj.DesiredLinearVelocity;
            %end
        end
        
        function dat = infoImpl(obj)
            %info Additional information about object status
            %   S = info(PP) returns a structure S with additional
            %   information about the current status of the PurePursuit
            %   object PP. The structure S contains the fields RobotPose,
            %   and LookaheadPoint. The RobotPose is the
            %   robot's pose that was the input to the last step function
            %   call. The LookaheadPoint is a point on the path that was 
            %   used to compute the outputs of the last step function call.
            %
            %   Example:
            %       % Create a PurePursuit object
            %       pp = robotics.PurePursuit;
            %
            %       % Assign waypoints
            %       pp.Waypoints = [0 0;1 1];
            %
            %       % Compute control commands
            %       [v, w] = step(pp, [0 0 0]);
            %
            %       % Get additional information
            %       s = info(pp)
            %
            %   See also robotics.PurePursuit
            
            dat.RobotPose = obj.LastPose;
            dat.LookaheadPoint = obj.LookaheadPoint;
        end
        
        function num = getNumInputsImpl(~)
            num = 1;
        end
        
        function num = getNumOutputsImpl(~)
            num = 2;
        end
        
        function s = saveObjectImpl(obj)
            %saveObjectImpl Custom save implementation
            s = saveObjectImpl@matlab.System(obj);
            
            s.Path = obj.Path;
            s.ProjPointIdx = obj.ProjPointIdx;
            s.DeltaDistance = obj.DeltaDistance;
            s.LookaheadPoint = obj.LookaheadPoint;
            s.LastPose = obj.LastPose;
            
            if isLocked(obj)
                s.Waypoints = obj.Waypoints;
            end
        end
        
        function loadObjectImpl(obj, svObj, wasLocked)
            %loadObjectImpl Custom load implementation
            
            obj.Path = svObj.Path;
            obj.ProjPointIdx = svObj.ProjPointIdx;
            obj.DeltaDistance = svObj.DeltaDistance;
            obj.LookaheadPoint = svObj.LookaheadPoint;
            obj.LastPose = svObj.LastPose;
            
            % Load state only if locked when saved
            if wasLocked
                obj.Waypoints = svObj.Waypoints;
            end
            
            % Call base class method
            loadObjectImpl@matlab.System(obj,svObj,wasLocked);
        end
        
        function resetImpl(obj)
            %resetImpl Reset the internal state to defaults
            obj.ProjPointIdx = 0;
            obj.LookaheadPoint = zeros(1,2);
            obj.LastPose = zeros(1,3);
        end
        
    end
    
    methods (Access = private)
        function avel = saturate(obj, avel)
            %saturate Saturate angular velocity
            
            if abs(avel) > obj.MaxAngularVelocity
                avel = sign(avel)*obj.MaxAngularVelocity;
            end
        end
        
        function nearIdx = findNearest(obj, pose)
            %findNearest Find closest point in the entire path
            dist = sqrt((obj.Path(:,1) - pose(1)).^2 + ...
                (obj.Path(:,2) - pose(2)).^2);
            [~,nearIdx] = min(dist);
        end
        
        function updateProj(obj, pose, lookAheadIdxDist)
            %updateProj Find closest point in next few points
            
            searchDist = 2*lookAheadIdxDist;
            if (obj.ProjPointIdx + searchDist < size(obj.Path, 1))
                lookForward = obj.ProjPointIdx + searchDist;
            else
                lookForward = size(obj.Path, 1);
            end
            
            dist = sqrt((obj.Path(obj.ProjPointIdx:lookForward,1) - pose(1)).^2 + ...
                (obj.Path(obj.ProjPointIdx:lookForward,2) - pose(2)).^2);
            
            [~,curProjection] = min(dist);
            
            obj.ProjPointIdx = obj.ProjPointIdx + curProjection - 1;
        end
    end
end
