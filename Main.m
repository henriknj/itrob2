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
    mydir = pwd;
    cd(toolboxPath3) 
    startup_rvc
    cd(mydir)
    
    % Main control sequence
    % Initial data
    if(verbose) 
        SMap.getInstance().showMap();
    end

    % ROS
    rosip = '192.168.1.100';
    myip = '192.168.1.101';
    robot = TurtleBot.getInstance();
    robot.connect(rosip,myip);
    
    % 011/013 to CompLab
     start = [4.72 	1.512];
     %start = [5.625 	1.512]; % til venstre for torben og heidis dør
     %start = [17.84 2.44];
     compLabViaPoints = [
         4.72	2.488+.4;
         % 5.625	2.488+.4; % til venstre for torben og heidis dør
         16.92	2.4;
         17.92-0.3  2.4;];
         %20      2.2;];
    % CompLab to 018
     elecLabViaPoints = [
         17.92  2.2;
         16.16	2.257;
         13.19-0.5	2.715;
         13.19   11.17;
         13.8    15.8+0.2;
         ];
    % 018 to CompLab
    %start = [12.36   18.64];
    %viapoints = [
    %   13.19   11.17;
    %   13.19	2.715;
    %   16.16	2.257;];

    % Start position
    pose = [start deg2rad(90)];
    
    % Setup robot controller
    controller = RobotController(pose);
    controller.ScanWeight = 1;
    controller.OdomWeight = 0.05;%0.06;

    % setup sensors
    robot.enableCamera();
    robot.enableSound();
    robot.enableBumper(controller);
    robot.enableCliff(controller);
    robot.enableOdom(controller);
    robot.enableLaser(controller);
    
    % Move to the componentlab (waypointx)
    controller.moveToPoint(compLabViaPoints);

    % Setup the marker locator
    detector = SegmentObjectDetector(controller,0.3,0.6,false);
    locator = MarkerLocator(detector,controller,0.40);
    markerPosition = locator.getMarkerPosition([17.92  2.4]);
    controller.getCurrentPose();
    
    % Move 40cm directly in front of marker
    controller.moveToPoint(markerPosition);
    
    % make a sound
    robot.setSound('turn on');
    
    % 
    controller.moveToOrientation(deg2rad(180));
    
    % Drive to the endgoal
    controller.moveToPoint(elecLabViaPoints);
    
catch e
    display(e);
    switch e.identifier
        case 'MarkerLocator:BadDetectorArgument'
            warning(e.msg);
    end
    
end

