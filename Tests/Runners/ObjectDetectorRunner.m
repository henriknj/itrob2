%
% Run tests
%
import matlab.unittest.TestSuite;
import matlab.unittest.constraints.IsTrue;

ObjectDetection = TestSuite.fromClass(?CircleDetectionTests);
result = run(ObjectDetection);