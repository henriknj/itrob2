% Include map
map = SMap.getInstance().getOccupancyGrid();

% Include ROB toolbox
start = [4.72 	1.512];
viapoints = [
    start;
    4.72	2.488;
    17.46	2.395;];

posCompLab = [17.92  2.2];
viaCompLab = [
    13.32	2.488;
    13.32	16.11;];

robotRadius = 0.2;
mapInflated = copy(map);
inflate(mapInflated,robotRadius);
prm = robotics.PRM(mapInflated);
prm.NumNodes = 200;
prm.ConnectionDistance = 10;

endLocation = [13.8    15.8+0.2];
path = findpath(prm, start, posCompLab)

figure
show(map)
hold on
show(prm, 'Map', 'off', 'Roadmap', 'on')
hold on
path = findpath(prm, posCompLab, endLocation)
show(prm, 'Map', 'off', 'Roadmap', 'off')
hold off