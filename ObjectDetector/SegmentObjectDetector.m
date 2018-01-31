%
% Image segmentation detector
%
classdef SegmentObjectDetector < ObjectDetector
    
    %SEGMENTOBJECTDETECTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        greycthredshold;
        eccentricity;
        controller;
        foundObject;
        fixedArea;
    end
    
    methods(Access=public)
        
        function obj = SegmentObjectDetector(ctrl, threshold,eccentricity,fixedArea)
            obj = obj@ObjectDetector();
            obj.greycthredshold = threshold;
            obj.eccentricity = eccentricity;
            obj.controller = ctrl;
            obj.foundObject = false;
            obj.fixedArea = fixedArea;
        end
        
        %
        % Locate object in image
        %
        function pose = findObject(obj,pointCloud) 
            global verbose;
            
            % Convert pcd to rgb image
            xrgb=reshape(pointCloud.readRGB(),pointCloud.Width,pointCloud.Height,3);
            xrgb=permute(xrgb, [2 1 3]);
            m1g = rgb2gray(xrgb);
            figure;
            imagesc(m1g), colorbar
            
            %
            % Otsu method for thresholding
            %
            if(obj.greycthredshold == -1)
               obj.greycthredshold = graythresh(xrgb); 
            end
            
            %
            % Segment the background from the object
            %
            m1b = m1g < obj.greycthredshold;
            if(verbose == 2)
                imagesc(m1b)
            end

            %
            % Perform morphological closing on greyscale image.
            % Create a disk-shaped structuring element to preserve
            % the circular nature of the object. The second parameter
            % specifies a radius of 5 pixels so that hte largest gaps get
            % filled.
            %
            m1bd = imclose(m1b, strel('disk', 5));
            if(verbose == 2);
                imagesc(m1bd)
            end

            % Extract properties from current image
            m1lab = bwlabel(m1bd);
            m1prop = regionprops(m1lab, 'Area', 'Eccentricity')

            % Calculate object size to find area
            xyz = pointCloud.readXYZ();

            if obj.fixedArea
                
                % Calculate object size to find area
                xyz = pointCloud.readXYZ();

                % get current pose
                curPose = obj.controller.getCurrentPose();

                % focal length in pixels (not 100% correct)
                k = 515; 
                % physical object size (8.5cm)
                x = 0.085; 
                % maximum values, we need distance
                [V,I] = max(xyz);

                % ~angle around 90 and 0 degrees with some threshold
                angleThres = 5;
                angleDegree = rad2deg(curPose(3));
                angleZero = angleDegree == 0 || angleDegree > 0+angleThres ...
                    || angleDegree < 0-angleThres;
                angleNine = angleDegree == 90 || angleDegree > 0+angleThres ...
                    || angleDegree < 0-angleThres;

                % Angle around 0,90 we just use the max distance
                % Else we take the half between max min
                if angleZero || angleNine
                    z = V(3); 
                else
                    z = abs(xyz(I(1),3)-V(3))/2 + 1;
                end                

                % Calculate a roughly estimate of the object size
                % in pixels
                estimatedArea = pi * ((k*(x/z))/2)^2;

                % We should maybe get a min or max value of a possible
                % area and use that instead.
                
                % Filter out objects, so we keep object in the interval.
                % [areaLower,areaHigher]
                areaLower = estimatedArea*1/5;
                areaHigher = estimatedArea + estimatedArea*1/3;
                
            else
                % fixed area
                areaLower = 600;
                areaHigher = 1600;
            end

            % Filter out the features with a given area 
            m1be = bwpropfilt(m1bd, 'Area', [areaLower areaHigher]); 
            if(verbose == 2)
                imagesc(m1be)
            end
            
            % Get region properties (Area,Eccentricity)
            m1lab = bwlabel(m1be);
            m1prop = regionprops(m1lab, 'Area', 'Eccentricity')
            
            if(verbose == 2)
                obj.drawCircles(m1prop);               
            end
            
            % 
            % Check region properties eccentricity
            %
            m1prop = regionprops(m1lab ,'Centroid',...
                    'MajorAxisLength','MinorAxisLength','Eccentricity');
            
            properties = struct2cell(m1prop);
           
            if length(m1prop) == 1
                if m1prop.Eccentricity < obj.eccentricity
                    obj.drawCircles(m1prop);
                    center = properties{1,1};
                    obj.foundObject = true;
                end
            else
                
                output = [];
                for k = 1:length(m1prop) 
                    output(k) = m1prop(k).Eccentricity;
                end

                for i=1:size(output,2)
                    if output(i) < obj.eccentricity
                        obj.drawCircles(m1prop,output(i));
                        center = properties{1,i};
                        obj.foundObject = true;
                    end
                end 
            end
            
            %
            % Find the position in pointcloud
            %
            if obj.foundObject               

                pcdIndex = pointCloud.Width * floor(center(2)) + floor(center(1));

                if(verbose)
                    %catter3(pointCloud);
                    %hold on;
                    %plot3(xyz(pcdIndex,1),xyz(pcdIndex,2), xyz(pcdIndex,3), 'magenta.');
                end

                % Pointcloud values
                pose = [];
                pose(1) = xyz(pcdIndex,1);            
                pose(2) = xyz(pcdIndex,2);
                pose(3) = xyz(pcdIndex,3);

            else
                pose = 0;
            end
            
        end
        
        function drawCircles(obj,region,varargin)
            % DRAWCIRCLES - Drawcircles on image
            %
            nVarargs = length(varargin);
            if nVarargs == 1
                if isfloat(varargin{1}) || isinteger(varargin{1}) ...
                        && varargin{1} <= 1.0 ...
                        && varargin{1} >= 0.0 ...
                        
                    objectToDrawIndex = find([region.Eccentricity] == varargin{1});
                    properties = struct2cell(region);

                    centers = properties{1,objectToDrawIndex};
                    majorAxisLength = properties{2,objectToDrawIndex};
                    minoAxisLength = properties{3,objectToDrawIndex};
                    diameters = mean([majorAxisLength minoAxisLength],2);
                    radii = diameters/2;
                    hold on
                    viscircles(centers,radii);
                    hold off   
                else
                    return;
                end
            else
                if nVarargs == 0
                    centers = region.Centroid;
                    diameters = mean([region.MajorAxisLength region.MinorAxisLength],2);
                    radii = diameters/2;
                    hold on
                    viscircles(centers,radii);
                    hold off     
                end
            end

     
        end
        
    end
    
end

