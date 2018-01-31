function [ posePredicted ] = posePrediction( scan_lines, robotPose, map )
% [ posePredicted ] = posePrediction( scan_lines, robotPose, map )
% Predict a pose based on the laser scan and map.
% NOTE: Assumes only rectangular wall-structures !! 

% To get a match between the laserscan and the map the difference is max:
rhoMatchMax = 2; % in meters
thetaMatchMax = 35; % in deg

ds = 0.0465/4; % step distance in meter 
max_scan_dis = 6.5; % max range of sensor
nMaxIter = max_scan_dis/ds;
sr = 1; % search range - in angle determination

Nx = map.XWorldLimits(2);
Ny = map.YWorldLimits(2);

noOfLines = size(scan_lines,2);
poseOffset = zeros(noOfLines, 3);
for k=1: noOfLines
    wallLine = scan_lines(k);

    % veh position
    x = robotPose(1);
    y = robotPose(2);
    th=-robotPose(3) + wallLine.searchAngle;
        
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
        elseif iter == nMaxIter,
            break;
        elseif getOccupancy(map, [x y]), % if hit - NOTE: reverse indexing (related to CORKE)
            bhit = 1;
            % Get a more precise measurement of where the wall starts.
			dx = double(dx/25);
            dy = double(dy/25);
            while getOccupancy(map, [(x-dx), (y-dy)]) == getOccupancy(map, [x, y])
                x = x - dx;
                y = y - dy;
            end
            break;
        end
        iter = iter+1;
    end
    
    % set default value
    correct_x = NaN;
    correct_y = NaN;
    correct_angle = NaN;  
    
    if bhit, % hit obstacle
        % finding range        
        xsum = 0;
        ysum = 0;      
        
        % Counting number of filled wall-elements in x-direction near xr
        for i=x-sr:x+sr,
            if i > 0 && i < Nx,
                xsum = xsum + getOccupancy(map, [i y]);
            end
        end
        for i=y-sr:y+sr,
            if i > 0 && i < Ny,
                ysum = ysum + getOccupancy(map, [x i]);
            end
        end

        if xsum == 2*sr+1, % horizontal wall
            %fprintf(' horizontal wall');      
            wall_vec = [1 0];
            % make a vector that is turned by 90 deg to robotPose angle.
            robot_vec = [-sind(-robotPose(3)), cosd(-robotPose(3))];
            % find the angle between wall_vec and robot_vec.
            alpha2 = acos(dot(wall_vec, robot_vec) / norm(wall_vec));
            alpha2 = rad2deg(alpha2);
            if robot_vec(2)<0
                alpha2 = -alpha2;
            end     
            % normalization to -pi/2 to pi/2
            alpha2 = atand(sind(alpha2)/cosd(alpha2));
            
            % orthogonal distance to wall. The easy way
            rho_map = abs(y-robotPose(2));
            % Check if the wall is matching the laser scan.
            if ( abs(wallLine.rho - rho_map) <  rhoMatchMax && ...
                 abs(abs(wallLine.theta) - abs(alpha2)) < thetaMatchMax )
             
                correct_y = rho_map - wallLine.rho;
                % check for if the wall is under the robot in the y-axis
                if y-robotPose(2) < 0
                    correct_y = -correct_y;
                end
                % cannot correct the x coordinate of a horizontal wall.
                %correct_x = NaN;
                
                % make -90 deg to 90 deg 
                if wallLine.theta == -90
                   wallLine.theta = 90; 
                end
                if alpha2 == -90
                   alpha2 = 90;
                end                
                correct_angle = -((wallLine.theta) - (alpha2));
                % Normalize to +- 90 deg
                correct_angle = atand(sind(correct_angle)/cosd(correct_angle));
            else
                fprintf(' could not match to map');    
            end
                               
        elseif ysum == 2*sr+1, % vertical wall
            %fprintf(' vertical wall');
            wall_vec = [0 1];
            % make a vector that is turned by 90 deg to robotPose angle.
            robot_vec = [-sind(-robotPose(3)), cosd(-robotPose(3))];
            % find the angle between wall_vec and robot_vec.
            alpha2 = acos(dot(wall_vec, robot_vec) / norm(wall_vec));
            alpha2 = rad2deg(alpha2);
            if robot_vec(1)>0
                alpha2 = -alpha2;
            end
            % normalization to -pi/2 to pi/2
            alpha2 = atand(sind(alpha2)/cosd(alpha2));
            
            % orthogonal distance to wall. The easy way
            rho_map = abs(x-robotPose(1));
            % Check if the wall is matching the laser scan.
            if ( abs(wallLine.rho - rho_map) <  rhoMatchMax && ...
                 abs(abs(wallLine.theta) - abs(alpha2)) < thetaMatchMax )
             
                correct_x = rho_map - wallLine.rho;
                % check for if the wall is to the left of the robot in the x-axis
                if x-robotPose(1) < 0
                    correct_x = -correct_x;
                end
                % cannot correct the y coordinate of a vertical wall.
                %correct_y = NaN; 
                
                % make -90 deg to 90 deg 
                if wallLine.theta == -90
                   wallLine.theta = 90; 
                end
                if alpha2 == -90
                   alpha2 = 90; 
                end                
                correct_angle = alpha2 - wallLine.theta;
                correct_angle = atand(sind(correct_angle)/cosd(correct_angle));           
            else
                fprintf(' could not match to map');
            end                    
        else
            % not horizontal and not vertical wall
            fprintf(' not horizontal and not vertical wall');
        end    
    else
        % did not hit obstacle!
        fprintf('did not hit obstacle!');
    end    

    poseOffset(k,1) = correct_x;
    poseOffset(k,2) = correct_y;
    poseOffset(k,3) = correct_angle;    
end

poseOffset
poseOffsetMean = nanmean(poseOffset); % mean ignoring NaN 
% change NaN values to zero
poseOffsetMean(isnan(poseOffsetMean)) = 0;
poseOffsetMean
posePredicted = robotPose + poseOffsetMean;
end
