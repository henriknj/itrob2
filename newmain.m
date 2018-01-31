%
% ITROB2 Robot
%
global verbose;

try
    % ROB toolbox + Machine vision !
    toolboxPath = '..\..\Toolbox\rvctools';
    toolboxPath3 = '..\rvctools';
    
    % Verbose
    verbose = true;
    
    % Add subfolders to path
    addpath(genpath(pwd));
    
    % Load robotics toolbox
    %mydir = pwd;
    %cd(toolboxPath3) 
    %startup_rvc
    %cd(mydir)
    
    % ROS
    rosip = '192.168.1.100';
    myip = '192.168.1.101';
    robot = TurtleBot.getInstance();
    robot.connect(rosip,myip);
    
    % Main control sequence
    % Initial data
    if(verbose) 
        SMap.getInstance().showMap();
    end
    % 011/013 to CompLab
     start = [4.72 	1.512];

    % Start position
    pose = [start deg2rad(180)];
    
    % Setup robot controller
    controller = RobotController(pose);
    robot.enableOdom(controller);
    controller.moveToOrientation(deg2rad(270));
    controller.moveToOrientation(deg2rad(90));
    controller.moveToOrientation(deg2rad(0));
    controller.moveToOrientation(deg2rad(45));
    controller.moveToOrientation(deg2rad(135));
    controller.moveToOrientation(deg2rad(315));
    controller.moveToOrientation(deg2rad(225));
    
catch e
    display(e);
    switch e.identifier
        case 'MarkerLocator:BadDetectorArgument'
            warning(e.msg);
    end
    
end

