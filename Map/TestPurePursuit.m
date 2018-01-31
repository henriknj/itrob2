MakeTraj2

pose = [start 0];
robotCurrentLocation = start;
robotGoal = path(end,:);%[13.32	16.11];

controller = robotics.PurePursuit;
controller.Waypoints = [path];
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 1;
controller.LookaheadDistance = 0.5;

goalRadius = 0.1;
distanceToGoal = norm(robotCurrentLocation - robotGoal);

robot = ExampleHelperDifferentialDriveRobot(pose);

while( distanceToGoal > goalRadius )

    % Compute the controller outputs, i.e., the inputs to the robot
    [vel, angvel] = step(controller, robot.CurrentPose);

    % Simulate the robot using the controller outputs.
    drive(robot, vel, angvel)

    % Extract current location information ([X,Y]) from the current pose of the
    % robot
    robotCurrentLocation = robot.CurrentPose(1:2);

    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentLocation - robotGoal);
end