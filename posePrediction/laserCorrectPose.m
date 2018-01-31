function [ offsetPose ] = laserCorrectPose( scan_lines, robotPose, map )
% [ posePredicted ] = laserCorrectPose( scan_lines, robotPose, map )
% Correct the current pose based on the map and lines from the laser scan.
% Return a offset pose.
% NOTE: Assumes only rectangular wall-structures in the map !! 

% To get a match between the laserscan and the map the difference is max:
rhoMatchMax = 1.5; % in meters
thetaMatchMax = 15; % in deg

mapRes = 1/map.Resolution; % get map resolution meters pr. pixel
ds = mapRes/1.5; % step distance in meter 
max_scan_dis = 4; % max range of sensor
nMaxIter = floor(max_scan_dis/ds);

Nx = map.XWorldLimits(2);
Ny = map.YWorldLimits(2);

noOfLines = size(scan_lines,2);
poseOffset = zeros(noOfLines, 3);
for k=1: noOfLines
    wallLine = scan_lines(k);

    % veh position
    x = robotPose(1);
    y = robotPose(2);
    th= rad2deg(robotPose(3)) + wallLine.searchAngle;
        
    iter = 0;
    bhit = 0;
    dx = double(ds*cosd(th));
    dy = double(ds*sind(th));
    while ~bhit, % until hit object     
        x = x + dx;
        y = y + dy;

        if x < 0 || x > Nx || y < 0 || y > Ny,
    %         xr, yr;
            break;
        elseif iter >= nMaxIter,
            break;
        elseif getOccupancy(map, [x y]), % if hit - NOTE: reverse indexing (related to CORKE)
            bhit = 1;
            % Get a more precise measurement of where the wall starts.
			dx = double(dx/10);
            dy = double(dy/10);
            while getOccupancy(map, [(x-dx), (y-dy)]) == getOccupancy(map, [x, y])
                x = x - dx;
                y = y - dy;
            end
            %xy = [x,y]
            break;
        end
        iter = iter+1;
    end
    
    % set default value
    correct_x = NaN;
    correct_y = NaN;
    correct_angle = NaN;     
    
    if bhit, % hit obstacle      
        horizontal = false;
        vertical = false;
        
        % determine if wall is vertical or horizontal
        if getOccupancy(map, [x-mapRes, y]) == false
           vertical = true;
        elseif getOccupancy(map, [x+mapRes, y]) == false
           vertical = true;
        end
        if getOccupancy(map, [x, y-mapRes]) == false
           horizontal = true;
        elseif getOccupancy(map, [x, y+mapRes]) == false
           horizontal = true;
        end
        
        % check if the wall is vertical and horizontal
        if vertical==true && horizontal==true
           horizontal = false;
           vertical = false;
        end

         if horizontal==true % horizontal wall
            %fprintf(' horizontal wall');             
            % make a vector that is turned by 90 deg to robotPose angle.
            robot_vec = [-sind(robotPose(3)), cosd(robotPose(3))];
            
            % make a wall vector that is in the same direction as robot_vec on the x-axis
            if robot_vec(1) < 0
                wall_vec = [-1 0];
            else
                wall_vec = [1 0];
            end
            
            % find the angle between wall_vec and robot_vec.
            theta = acos(dot(wall_vec, robot_vec) / norm(wall_vec));
            theta = rad2deg(theta);
            
            if wall_vec(1) == -1
                if robot_vec(2)<0
                    theta = -theta;
                end
            else
                if robot_vec(2)>0
                    theta = -theta;
                end
            end

            % make -90 deg to 90 deg 
            if wallLine.theta == -90
               wallLine.theta = 90; 
            end
            if theta == -90
               theta = 90;
            end

            theta = wallLine.theta - theta;
            % normalize to +-90 deg
            theta = atand(sind(theta)/cosd(theta));
            
            % orthogonal distance to wall. The easy way
            rho_map = y-robotPose(2);
            % Check if the wall is matching the laser scan.
            if ( abs(wallLine.rho - abs(rho_map)) <  rhoMatchMax && ...
                 abs(theta) < thetaMatchMax )
             
                % check for if the wall is under the robot in the y-axis
                if rho_map < 0
                    wallLine.rho = -wallLine.rho;
                end    
                correct_y = rho_map - wallLine.rho;             

                % cannot correct the x coordinate of a horizontal wall.
                %correct_x = NaN;
                
                correct_angle = -theta;
            else
                %fprintf(' could not match to map\n');    
            end
                               
         elseif vertical==true % vertical wall
            %fprintf(' vertical wall');            
            % make a vector that is turned by 90 deg to robotPose angle.
            robot_vec = [-sind(robotPose(3)), cosd(robotPose(3))];
            
            % make a wall vector that is in the same direction as robot_vec on the y-axis
            if robot_vec(2) < 0
                wall_vec = [0 -1];
            else
                wall_vec = [0 1];
            end
            
            % find the angle between wall_vec and robot_vec.
            theta = acos(dot(wall_vec, robot_vec) / norm(wall_vec));
            theta = rad2deg(theta);
            
            if wall_vec(2) == -1
                if robot_vec(1)>0
                    theta = -theta;
                end
            else
                if robot_vec(1)<0
                    theta = -theta;
                end
            end             
            
            % make -90 deg to 90 deg 
            if wallLine.theta == -90
               wallLine.theta = 90; 
            end
            if theta == -90
               theta = 90; 
            end                
            theta = wallLine.theta - theta;
            % normalize to +-90 deg
            theta = atand(sind(theta)/cosd(theta));			
            
            % orthogonal distance to wall. The easy way
            rho_map = x-robotPose(1);
            % Check if the wall is matching the laser scan.
            if ( abs(wallLine.rho - abs(rho_map)) <  rhoMatchMax && ...
                 abs(theta) < thetaMatchMax )
             
                % check for if the wall is to the left of the robot in the x-axis
                if rho_map < 0
                    wallLine.rho = -wallLine.rho;
                end    
                correct_x = rho_map - wallLine.rho;
                
                % cannot correct the y coordinate of a vertical wall.
                %correct_y = NaN; 
                
                correct_angle = -theta;           
            else
                %fprintf(' could not match to map\n');
            end                    
        else
            % not horizontal and not vertical wall
            %fprintf(' not horizontal and not vertical wall\n');
        end    
    else
        % did not hit obstacle!
        %fprintf('did not hit obstacle!\n');
    end    

    poseOffset(k,1) = correct_x;
    poseOffset(k,2) = correct_y;
    poseOffset(k,3) = deg2rad(correct_angle);    
end

poseOffset
poseOffsetMean = nanmean(poseOffset, 1); % mean ignoring NaN 
offsetPose = poseOffsetMean;
end

