clear; close all; clc;
addpath(genpath(pwd));

% Parameters
RotateFrom = 180;
RotateTo = -90;
OdomOrientation = 0;

% Find rotation angle
pose = [0 0 deg2rad(RotateFrom)];
currentOrientation = pose(3);
orientation = deg2rad(RotateTo);

rotationAngle = orientation-currentOrientation;
maxAngularVelocity = 2;

% Get odometry pose
theta = deg2rad(OdomOrientation);
theta_goal = theta+rotationAngle;

timestep = 0.1;
angularVel = maxAngularVelocity/2;

% Calculate error
previous_error = 0;
integral = 0;
if theta > theta_goal
    error = theta - theta_goal
else
    error = theta_goal - theta
end
Kp = 1;
Ki = 0.009;
Kd = 0.1;

figure(99), hold on, axis([-pi,pi,-pi,pi]);
drawRobotToMap([pose(1:2) rad2deg(pose(3))]);
PosePlot = [];
HeadingPlot = [];

while abs(error) > deg2rad(0.2)
    % Get error
    error = theta_goal - theta
    integral = error*timestep + integral;
    derivative = (error - previous_error)/timestep;

    output_vel = Kp*error*angularVel+Ki*integral+Kd*derivative+sign(error)*angularVel*0.1;
    if output_vel > maxAngularVelocity/2
        output_vel = maxAngularVelocity/2;
    end

    pause(timestep);
    
    previous_error = error;
    theta = theta + output_vel*timestep;
    pose = [pose(1:2) theta];
    
    if ~isempty(PosePlot)
        delete(PosePlot);
        delete(HeadingPlot);
    end
    [PosePlot, HeadingPlot ] = ...
        drawRobotToMap([pose(1:2) rad2deg(pose(3))]);
end