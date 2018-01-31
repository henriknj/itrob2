% Contrast
warning('off','all');
global verbose;
verbose = true;
robot = RobotController([0 0 0]);
%detector = CircleObjectDetector(4,10,'dark',0.89);  
detector = SegmentObjectDetector(robot,0.3,0.65);
%load('Data\componentlab\wall2\waypoint1_rightAngle_80_markerBot.mat');
%load('Data\componentlab\wall2\waypoint1_rightAngle_80_markerCenter.mat');
%load('Data\componentlab\wall2\waypoint1_rightAngle_80_markerTop.mat');

% Some issues as it removes the circle almost.
%load('Data\componentlab\wall2\waypoint1_leftAngle_100_markerBot.mat');
%load('Data\componentlab\wall2\waypoint1_leftAngle_100_markerCenter.mat');%
load('Data\componentlab\wall2\waypoint1_leftAngle_100_markerTopInvisible.mat');%
%load('Data\componentlab\wall1\door_center.mat');


objectPosition = detector.findObject(pcd);