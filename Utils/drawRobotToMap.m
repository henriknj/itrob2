function [handlePose, handleLine] = drawRobotToMap( robotPose )
% function [ ] = drawRobotToMap( robotPose, map, handle )
%   Detailed explanation goes here

xPose = robotPose(1);
yPose = robotPose(2);
th = robotPose(3);

figure (99), hold on
handlePose = plot(xPose, yPose, 'ko', 'MarkerSize', 10);
lx = [xPose, xPose + 2*cosd(th)];
ly = [yPose, yPose+ 2*sind(th)];
handleLine = line(lx, ly, 'Color','r', 'LineWidth', 2);

end

