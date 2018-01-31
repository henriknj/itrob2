function [ output_lines ] = lineExtraction( laserscan, showFigure, showImage )
% [ output_lines ] = lineExtraction( laserscan, showFigure, showImage )
% Deception: Extract the lines in the in the laserscan.
% laserscan: raw laserscan
% showFigure: bool value, true to show figure
% showImage: bool value, true to show image

%% Make scan data to x,y coordinates
% find delta rad between each scan point
delta_rad = ((laserscan.AngleMax)*2)/size(laserscan.Ranges, 1); 
angle = laserscan.AngleMax;
points = zeros(size(laserscan.Ranges, 1),2);
for grNo = 1:size(laserscan.Ranges, 1)
    if ( isnan(laserscan.Ranges(grNo)) == false )
        y = sin(angle)*laserscan.Ranges(grNo);
        x = cos(angle)*laserscan.Ranges(grNo);
    else
        y = NaN;
        x = NaN;
    end
    points(grNo,1) = x;
    points(grNo,2) = y;
    angle = angle - delta_rad;
end
points = points(~isnan(points(:,1)),:); % remove NaN
% Remove scan points longer than 4m away
points = points(points(:,1)<4,:);

pointY = points(:,1);
pointX = points(:,2);

%% Make scanpoints to binary image
y_max = max(pointY);
y_min = min(pointY);
x_max = max(pointX);
x_min = min(pointX);

% find midrange of y-points
y_midrange = (y_max-y_min)/2;

% Set min distance to 1.6m
if y_midrange < 1.6
    y_midrange = 1.6;
end

% The equation is found by measuring the y-axis resolution at different
% distances, and made a exp fit on the points
y_delta = 0.04539*exp(0.21248*y_midrange)-0.05808;
y_delta = y_delta*1.00;
% find colloms of image
imageCol = ceil((y_max - y_min)/y_delta);

x_delta = tan(delta_rad)*y_midrange;
x_delta = x_delta*4;
imageRow = ceil((x_max - x_min)/x_delta);

BWimage = false(imageCol, imageRow); %Make black image
% Insert points to image
for grNo=1 :size(points, 1)
   y = imageCol - round((points(grNo, 1)-y_min)/y_delta) +1;
   x = round((points(grNo, 2)-x_min)/x_delta)+1;
   BWimage(y, x) = true;
end
%% Make Hough transform and find the lines in the images
% Create the Hough transform 
[H,theta,rho] = hough(BWimage,'RhoResolution',2,'ThetaResolution',4);

% if showFigure==true
%     figure, imshow(imadjust(mat2gray(H)),[],'XData',theta,'YData',rho,...
%            'InitialMagnification','fit');
%     xlabel('\theta (degrees)'), ylabel('\rho');
%     axis on, axis normal, hold on;
%     colormap(hot)
% end

[NHoodSizeX, NHoodSizeY] = size(H);
if NHoodSizeX > 67
    NHoodSizeX = 67;
end
if NHoodSizeY > 45
    NHoodSizeY = 45;
end
%check for enven values of size X & Y
if mod(NHoodSizeX,2) == 0
  % number is even make it to odd.
  NHoodSizeX = NHoodSizeX-1;  
end
if mod(NHoodSizeY,2) == 0
  % number is even make it to odd.
  NHoodSizeY = NHoodSizeY-1;  
end 
% Find peaks in the Hough transform of the image.
P = houghpeaks(H,7,'threshold',ceil(.2*max(H(:))), 'NHoodSize', [NHoodSizeX,NHoodSizeY]);

% if showFigure==true
%     x = theta(P(:,2));
%     y = rho(P(:,1));
%     plot(x,y,'s','color','black');
% end

% extract lines
fillGab = double(ceil(imageRow/6));
max_p = double(max(imageCol, imageRow));
lines = houghlines(BWimage,theta,rho,P,'FillGap', fillGab,'MinLength', 37);


if showImage==true
	clf(101)
    figure (101), imshow(BWimage), hold on
end
% Create a plot that superimposes the lines on the original image.
for k = 1:length(lines)   
    if showImage==true
       xy = [lines(k).point1; lines(k).point2];
       plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','cyan');

       % Plot beginnings and ends of lines
       plot(xy(1,1),xy(1,2),'x','LineWidth',3,'Color','green');
       plot(xy(2,1),xy(2,2),'x','LineWidth',3,'Color','red');
    end
end

%% Plot lines in scatter plot and find theta,rho and search angle of the lines.
if showFigure==true
    figure(100);
	clf(100)
    figure (100), scatter( pointX, pointY), hold on
end
[lines(:).searchAngle] = deal(zeros(1,1));
for k = 1:length(lines)
    %convert pixel value to x,y coordinat system.
    start_pointy = -(lines(k).point1(2)-1-imageCol) * y_delta + y_min; 
    start_pointx = (lines(k).point1(1)-1) * x_delta + x_min;   
    lines(k).point1 = [start_pointx, start_pointy];
    end_pointy = -(lines(k).point2(2)-1-imageCol) * y_delta + y_min;
    end_pointx = (lines(k).point2(1)-1) * x_delta + x_min;
    lines(k).point2 = [end_pointx, end_pointy];
    
    if showFigure==true
        plot([start_pointx,end_pointx],[start_pointy,end_pointy],'LineWidth',2,'Color','cyan');
    end
    
    % orthogonal distance to wall
    lines(k).rho = abs(end_pointx*start_pointy-end_pointy*start_pointx)...
           / ((end_pointy-start_pointy)^2+(end_pointx-start_pointx)^2)^0.5;

    % Compute the point in the middle of the line
    wallMiddlePoint = [.5*(end_pointx-start_pointx)+start_pointx,...
                       .5*(end_pointy-start_pointy)+start_pointy];
    if showFigure==true
        plot(wallMiddlePoint(1), wallMiddlePoint(2),'x','LineWidth',3,'Color','red');
    end
    
    % Compute sceach wall angle
    nomVek = [0 1];
    sceachWallAngle = acosd(dot(wallMiddlePoint, nomVek) / norm(wallMiddlePoint));
    if wallMiddlePoint(1)>0
        sceachWallAngle = -sceachWallAngle;
    end
    lines(k).searchAngle = sceachWallAngle;
    
    % Compute line angle to robot
    wallVek = lines(k).point2 - lines(k).point1;
    nomVek = [1 0];
    theta = acosd(dot(wallVek, nomVek) / norm(wallVek));
    if wallVek(2)<0
    theta = -theta;
    end
    lines(k).theta = theta;    
end

output_lines = lines;

end

