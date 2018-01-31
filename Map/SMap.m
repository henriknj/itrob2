%
% Robot worldmap
%
classdef SMap < handle
    %MAP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access=private)
        map;
        mappath;
    end
    
    methods (Access=private)
        % Guard the constructor against external invocation.
        % We only want to allow a single instance of this class
        % This is ensured by calling the constructor from the static (non-class) getInstance() function
        function obj = SMap()
            % This file make image to map
            obj.mappath = 'map_v3.png';
            obj.createMap();
        end
        
        function createMap(obj)
            % This file make image to map
            mapIm = rgb2gray(imread(obj.mappath));

            % Make image to bw image.
            mapIm = mapIm < 10;
            %figure, imshow(mapIm);

            % Resize image
            %mapIm = imresize(mapIm, 0.5, 'nearest');
            %figure, imshow(mapIm);

            % Make Map
            %meters_pr_pix = 0.0465;
            meters_pr_pix = 0.02291139240506329113924050632911;
            obj.map = robotics.BinaryOccupancyGrid(mapIm,1/meters_pr_pix);
        end
    end
    
    methods (Access=public)
        
        function occupied = isPointOccupied(obj,xy)
            occval = getOccupancy(obj.map,xy);
            if(occval == 1)
                occupied = true;
            end
        end
        
        function setMap(obj,path)
            obj.mappath = path;
            obj.createMap();
        end
        
        function showMap(obj)
            figure(99), show(obj.map)
        end
        
        function grid = getOccupancyGrid(obj)
            grid = obj.map;
        end
    end
     
    methods (Static)  % Access=public
       function obj = getInstance()
            persistent uniqueInstance
            if isempty(uniqueInstance)
                obj = SMap();
                uniqueInstance = obj;
            else
                obj = uniqueInstance;
            end
        end 
    end
    
end

