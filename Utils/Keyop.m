rosip = '192.168.1.105';
myip = '192.168.1.101';
robot = TurtleBot.getInstance();
robot.connect(rosip,myip);

tbot = KeyopCommunicator();

enableOdom(tbot);
enableLaser(tbot);

OdomSub = rossubscriber('/odom', 'BufferSize', 25);
msg = receive(OdomSub,3);
disp('Successfully Enabled Odometry');

pause(2)

exampleHelperTurtleBotKeyboardControl(tbot);