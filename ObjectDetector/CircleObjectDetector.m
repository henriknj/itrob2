%
% CircleObjectDetector
%
classdef CircleObjectDetector < ObjectDetector
    %ObjectDetector Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        objectRadiusMin;
        objectRadiusMax;
        objectPolarity;
        objectSensitivity;
        rgbImg;
    end
    
    methods(Access = public)
                
        function obj = CircleObjectDetector(radMin,radMax,polarity,sensitivity)
            obj = obj@ObjectDetector();
            obj.objectRadiusMin = radMin;
            obj.objectRadiusMax = radMax;
            obj.objectPolarity = polarity;
            obj.objectSensitivity = sensitivity;
        end
       
        function setRadius(obj,min,max)
           obj.objectRadiusMin = min; 
           obj.objectRadiusMax = max;
        end
        
        function setSensitivity(obj,sense)
            obj.objectSensitivity = sense;
        end
        
        %
        % Locate object in image
        %
        function pose = findObject(obj,pointCloud,dimension) 
            global verbose;

            % Convert pointcloud to rgb
            xyz = pointCloud.readXYZ();            
            obj.rgbImg = reshape(pointCloud.readRGB(),pointCloud.Width,pointCloud.Height,3);
            obj.rgbImg=permute(obj.rgbImg, [2 1 3]);
                                    
            % Locate center of circle
            [center, radii] = imfindcircles(obj.rgbImg,...
                [obj.objectRadiusMin obj.objectRadiusMax],...
                'ObjectPolarity',obj.objectPolarity,...
                'Sensitivity',obj.objectSensitivity);
                         
            % in case of multiple objects
            % pick the one closest
            if(size(center) < 1)
                pose = [];
                return;
            else
                centerIndex = 1;
                smallestMean = 0;
                for i = 1:size(center,1)
                    
                    % get mean of area
                    meanArea = obj.circleDifference(center(i,1),center(i,2),radii(i));                     
                    contrast = obj.regionContrast(center(i,1),center(i,2),radii(i));
                    
                    % Limit area
                    % If not working
                    
                    if(meanArea > smallestMean)
                        smallestMean = meanArea;
                        centerIndex = i;
                    end
                end
                
                %
                % The circle is probably the one with the smallest 
                % mean value
                %
                center = center(centerIndex,:);
                radii = radii(centerIndex);
            end
               
            if strcmp('2d', dimension)
               pose = center;
               pose(size(pose,2)+1) = radii;
            else
                % Find the index in the pointcloud
                pcdIndex = pointCloud.Width * floor(center(2)) + floor(center(1));
                
                % Pointcloud values
                pose = [];
                pose(1) = xyz(pcdIndex,1);            
                pose(2) = xyz(pcdIndex,2);
                pose(3) = xyz(pcdIndex,3);              
            end

            if(verbose)
                scatter3(pointCloud);
                hold on;
                plot3(xyz(pcdIndex,1),xyz(pcdIndex,2), xyz(pcdIndex,3), 'magenta.');
                figure;
                imshow(obj.rgbImg);
                viscircles(center,radii);
            end

        end
        
    end
    
    methods(Access = private)
        
        function [img,mask] = areaMask(obj,xc,yc,r)
            % find min/max values
            [sy sx] = size(obj.rgbImg);
            xmin = max(1, floor(xc-r));
            xmax = min(sx, ceil(xc+r));
            ymin = max(1, floor(yc-r));
            ymax = min(sy, ceil(yc+r));
            
            % trim boundaries of the image
            % to only be the area in xmin,max,ymin,max
            img = obj.rgbImg(ymin:ymax, xmin:xmax); 
            xc = xc - xmin + 1;
            yc = yc - ymin + 1;
            
            % Make a circle mask
            [x, y] = meshgrid(1:size(img,2), 1:size(img,1));
            mask = (x-xc).^2 + (y-yc).^2 < r.^2;      
        end
        
        %
        % Get the contrast in area
        %
        function contrast = regionContrast(obj,xc,yc,r)           
            % mask
            [img,mask] = obj.areaMask(xc,yc,r);
            
            figure, imhist(img)
            
            % Compute mean
            intensity = double(img) .* mask;
            [maxVal,~] = max(intensity(:));
            [minVal,~] = min(intensity(:));
            contrast = maxVal - minVal;
        end
        
        %
        % Get the mean value of the area
        %
        function mean = regionMeanValue(obj,xc,yc,r)
            
            % mask
            [img,mask] = obj.areaMask(xc,yc,r);
                        
            % Compute mean
            mean = sum(sum(double(img) .* mask)) / sum(mask(:));
            
        end
        
        %
        % Get the difference between circle and outer circle
        %
        function outerRegion = circleDifference(obj,xc, yc, r)
            
            circleMean = obj.regionMeanValue(xc,yc,r);
            outercircleMean = obj.regionMeanValue(xc,yc,r+20);
            
            outerRegion = outercircleMean - circleMean; 
        end        
        
    end
end

