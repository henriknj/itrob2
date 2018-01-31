% Include map
map = SMap.getInstance().getOccupancyGrid();

% Include ROB toolbox
mydir = pwd;
cd('..\..\rvctools') % ROB toolbox + Machine vision !
startup_rvc
cd(mydir)

start = [4.72 	1.512];
compLabViaPoints = [
         start;
         4.72	2.488;
         16.92	2.2;
         17.92  2.2;];

posCompLab = [17.92  2.2];
elecLabViaPoints = [
         17.92  2.2;
         16.16	2.257;
         13.19	2.715;
         13.19   11.17;
         13.8    15.8+0.2;
         ];

% Smoothed waypoints - trajectory
traj = mstraj(compLabViaPoints, [2 2], [], start, 0.5, 1);
traj2 = mstraj(elecLabViaPoints, [2 2], [], posCompLab, 0.5, 1);

% Velocities based on trajectory
sv = diff(traj); % smoothed velocity (proportional)

% Headings based on trajectory
th = atan2( sv(:,2), sv(:,1)); % angles theta
th = [th; th(end,:)];

% Pose trajectory
posetraj = [traj th];

figure
show(map)
hold on
plot(traj(:,1), traj(:,2))
hold on
plot(traj2(:,1), traj2(:,2))