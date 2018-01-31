%
% Run tests
%
addpath(genpath(strcat(pwd, '/../')));
import matlab.unittest.TestSuite;

ObjectDetection = TestSuite.fromClass(?CircleDetectionTests);
MarkerLocator = TestSuite.fromClass(?MarkerLocatorTests);
TurtleBot = TestSuite.fromClass(?TurtleBotTests);
PoseCorrection = TestSuite.fromClass(?PoseCorrectionTests);
RobotCtrlTests = TestSuite.fromClass(?RobotControllerTests);


largeSuite = [TurtleBot];

result = run(largeSuite);