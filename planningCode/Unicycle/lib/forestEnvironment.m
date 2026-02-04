% The MIT License (MIT)
%
% Copyright 2022 Mohamed Khalid M Jaffar, University of Maryland
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% The software is provided "As Is", without warranty of any kind, express or
% implied, including but not limited to the warranties of merchantability,
% fitness for a particular purpose and noninfringement. In no event shall the
% authors or copyright holders be liable for any claim, damages or other
% liability, whether in an action of contract, tort or otherwise, arising from,
% out of or in connection with the software or the use or other dealings in
% the software.

%Class definition for the random forest environment with collision-checking routines
classdef forestEnvironment < handle
    properties     
        
        environmentType
        envLB_x
        envUB_x
        envLB_y
        envUB_y
        numObstacles %useful for keeping track of num of active obstacles
        dynamicity %percentage of obstacles changing in location and size
        toleranceLimit
        sensorRadius %sensor radius of the robot
        sizeRange %size range of circular obstacles
        mode %options: 'sensing' or 'dynamic' (addition and deletion)
        
        %list of obstacles with obstacleStruct datatype
        boundingRectangles
        obstacles
        obstacleTree
     
        %internal use
        indexOfLast        
    end

    methods
        %constructor class - initialises with the position, size and an unique id
        function obj = forestEnvironment(envLB_x,envUB_x,envLB_y,envUB_y,sensorRadius,epsilon,sizeRange,mode,vargin)
            
            obj.environmentType = 'forest';
            obj.envLB_x = envLB_x; obj.envUB_x = envUB_x;
            obj.envLB_y = envLB_y; obj.envUB_y = envUB_y;
            obj.numObstacles = 0;
            obj.indexOfLast = 0;

            obj.sensorRadius = sensorRadius; %14
            %obj.toleranceLimit = epsilon/2; %extra-padding
            obj.toleranceLimit = 1.5;
            obj.sizeRange = sizeRange; %specify the size range of circular obstacles
            obj.mode = mode;

            if strcmpi(obj.mode, 'sensing')
                obj.dynamicity = 0; %no obstacles get deleted
            else
                obj.dynamicity = vargin(1);
                return %no need of having a kDTree of obstacles (for dynamic environment)
            end
            
            distFunct = @(inputA, inputB) sqrt(sum((inputA - inputB).^2,2)); %distance function
            obj.obstacleTree = KDTree(2, distFunct); %initialise the tree, 2 - num of dimensions
        end
        
        %whenever some changes happen to the underlying data structure
        %it needs to be returned, hence have to include "obj = funcName()"
        
        %functions to add and remove obstacles to the list
        function obj = addObstacle(obj,obstacle)
            
            if isnan(obstacle.index)
                obstacle.index = obj.indexOfLast+1;
            end
            obj.obstacles{end+1} = obstacle;
            obj.indexOfLast = obj.indexOfLast+1;
            obj.numObstacles = obj.numObstacles+1; %increment the total #obstacles by 1
        end
        
        function obj = initialiseObstacleTree(obj)

            for i=1:obj.indexOfLast
                thisObstacle = obj.obstacles{i};
                
                if strcmpi(obj.mode, 'sensing')
                    thisObstacle.status = 0; %at the start, all the obstacles are unexplored
                else
                    thisObstacle.status = 1; %at the start, all the obstacles explored (in dynamic environment)
                end
                
                %add to the obstacle kdTree
                tempKDnode = KDTreeNode(thisObstacle.location); %changing it to pose instead of position
                tempKDnode.payload = thisObstacle;
                obj.obstacleTree.kdInsert(tempKDnode);
            end  
        end

        function exploredObstacles = senseObstacles(obj,robotLocation)
            
            exploredObstacles = {};
            if obj.numObstacles == 0
                return %return if obstacle-free
            end
            
            tempKDTree = obj.obstacleTree;
            %range = 1.05*obj.sensorRadius; %trying to make up for circleRadius
            range = obj.sensorRadius + obj.toleranceLimit;

            obstaclesInRange = tempKDTree.kdFindWithinRangePayload(range, robotLocation);            
            
            for i=1:length(obstaclesInRange)
                tempObstacle = obstaclesInRange{i};
                
                if tempObstacle.status == 1 %if it's already explored, continue
                    continue
                end
                
                exploredObstacles{end+1} = tempObstacle;
                tempObstacle.status = 1;
            end
            
        end
        
        function addedObstacles = addDynamicObstacles(obj,numObstacles,robotPose,goalPose,varargin) %n - number of obstacles
            
            addedObstacles = cell(3*numObstacles,1); %assuming each car is represented as 3 bounding-circle obstacles

            count = 1;
            while count<=numObstacles
                
                if isempty(varargin) %if location is not specified
                    location = zeros(1,2);
                    location(1) = round(rand()*(obj.envUB_x - obj.envLB_x) + obj.envLB_x); %rounding off because car-obstacles can exist only in the lanes
                    location(2) = rand()*(obj.envUB_y - obj.envLB_y) + obj.envLB_y;
                else
                    randRadius = rand()*obj.sensorRadius;
                    randTheta = rand()*2*pi;
                    location = [robotPose(1) + randRadius*cos(randTheta), robotPose(2) + randRadius*sin(randTheta)];  
                end
                
                radius = obj.sizeRange(1) + (obj.sizeRange(2) - obj.sizeRange(1))*rand();

                if (obj.euclidianDist(location,goalPose) < radius+obj.sensorRadius/2) || ...
                        (obj.euclidianDist(location,robotPose) < radius+obj.sensorRadius/2)
                    continue %explicitly avoid obstacles occluding start or goal location
                end
                
                width = radius*sqrt(2); length = 3*width;
                obj.boundingRectangles{count} = rectangleStruct(count, location, [width length], 0);

                rectangleObstacleLocation_x = location(1);
                rectangleObstacleLocation_y = [location(2) - width, location(2), location(2) + width];
                %initialise an obstacle and add it to the list
                for i=1:3 %
                    tempLocation = [rectangleObstacleLocation_x rectangleObstacleLocation_y(i)];
                    randomObstacle = obstacleStruct(obj.indexOfLast+1,tempLocation,radius);
                    obj.addObstacle(randomObstacle);
                    addedObstacles{count+i-1} = randomObstacle;
                end
                count = count+1;
            end
            
            % Not relevant for this system experiments
            % if strcmpi(obj.mode, 'dynamic')
            % 
            %     % make m<=N obstacles inactive at random
            %     %at random make (numObstacles * obj.dynamicity/100)
            %     %obstacles active (status = 0)
            % 
            %     %Options: 0. none 1. all obstacles, 2. half of the obtacles (preferred), 
            %     % 3. number of obstacles that would change
            %     numInactiveObstacles = 0;
            %     %numInactiveObstacles = obj.indexOfLast;
            %     %numInactiveObstacles = ceil(obj.indexOfLast/2);
            %     %numInactiveObstacles = numObstacles - ceil(numObstacles * obj.dynamicity/100);
            %     randomIndices = randperm(numObstacles, numInactiveObstacles);
            % 
            %     for i=1:length(randomIndices)
            %         obj.obstacles{i}.status = 0;
            %     end
            % end
 
        end
        
        % function addedObstacles = addRandomObstacles(obj,robotPose,goalPose,n) %n - number of obstacles
        % 
        %     addedObstacles = cell(n,1);
        %     %offset = obj.sensorRadius/2; %making sure that obstacles don't get added on the robot itself
        %     offset = obj.toleranceLimit + obj.sizeRange(2); %making sure that obstacles don't get added on the robot itself
        % 
        %     for i=1:n
        %         %assign random locations and size within sensor radius
        %         randRadius = (obj.sensorRadius-offset)*rand() + offset;
        %         %randRadius = (3*obj.sensorRadius-offset)*rand() + offset;
        %         randTheta  = 2*pi*rand();
        %         location = [robotPose(1)+randRadius*cos(randTheta), robotPose(2)+randRadius*sin(randTheta)];
        % 
        %         size = obj.sizeRange(1) + (obj.sizeRange(2) - obj.sizeRange(1))*rand();
        % 
        %         if (obj.euclidianDist(location,goalPose) < size+obj.sensorRadius/2) || ...
        %                 (obj.euclidianDist(location,robotPose) < size+obj.sensorRadius/2)
        %             continue %explicitly avoid obstacles occluding start or goal location
        %         end
        % 
        %         %initialise an obstacle and add it to the list
        %         randomObstacle = obstacleStruct(obj.indexOfLast+1,location,size);
        %         obj.addObstacle(randomObstacle);
        %         addedObstacles{i} = randomObstacle;
        %     end     
        % end
        
        
        % function obj = removeThisObstacle(obj,F,G,obstacle)
        % 
        %     if(obj.numObstacles < 1 || obstacle.status == 0)
        %         return %no obstacle to remove or if obstacle has already been removed
        %     end
        % 
        %     obstacle.status = 0;
        %     obj.numObstacles = obj.numObstacles-1; %decrement the total #obstacles by 1
        % 
        %     tempNodes = obstacle.nodesWithin;
        %     for i=1:length(tempNodes)
        %         tempNodes{i}.withinObstacle = 0;
        %     end
        % 
        %     tempEdges = obstacle.edgesWithin;
        %     for i=1:length(tempEdges)
        %         thisEdge = tempEdges(i);
        % 
        %         head = G.graphVertices(thisEdge.parent);
        %         %tail = C.graphVertices(thisEdge.child);
        %         %newCost = euclidianDist(obj,head.pose,tail.pose);
        % 
        %         thisEdge.withinObstacle = 0;
        %         thisEdge.cost = thisEdge.nominalCost; %reset to the original non-infinite cost
        % 
        %         %tempEdges(i).withinObstacle = 0;
        %         %tempEdges(i).cost = tempEdges(i).nominalCost;
        % 
        %         %head.vertexData(2) corresponds to the funnel-edge
        %         %so should be tail.vertexData(2) (by construction)
        %         thisFunnel = F.funnelEdges(head.vertexData(2));
        %         thisFunnel.cost = thisFunnel.nominalCost; %reset to the original non-infinite cost
        %         thisFunnel.withinObstacle = 0;
        %     end      
        % end
        % 
        % 
        % function deletedObstacles = removeRandomObstacles(obj,F,G,n)
        % 
        %     % if nargin < 4
        %     %     n = ceil(obj.numObstacles * obj.dynamicity/100);
        %     % end
        % 
        %     deletedObstacles = cell(n,1);
        %     count = 0;
        %     while count < min(n,obj.numObstacles) 
        %         index = randi([1 obj.indexOfLast]);
        %         tempObstacle = obj.obstacles{index};
        % 
        %         if(tempObstacle.status == 0) 
        %            continue %if the obstacle is inactive continue
        %         end
        % 
        %         obj.removeThisObstacle(F,G,tempObstacle);
        %         count = count+1;
        %         deletedObstacles{count} = tempObstacle;
        %     end
        % 
        % end
        
        %functions to determine which nodes and edges are within obstacles
        function collisionNodes = findNodesWithinObstacles(obj,C,tree,obstacles)
            
            collisionNodes = [];
             for i = 1:length(obstacles)

                if isempty(obstacles{i}) %just in case
                    continue
                end

                nodes = findNodesWithinEachObstacle(obj,C,tree,obstacles{i});   
                collisionNodes = [collisionNodes nodes'];
            end
        end
        
        
        function nodes = findNodesWithinEachObstacle(obj,C,tree,obstacle)

            if obstacle.status == 0 
                nodes = {};
                return %if the obstacle is not active continue
            end

            centre  = obstacle.location;
            epsilon = obstacle.radius; 

            nodes = tree.kdFindWithinRangePayload(epsilon,centre);
            obstacle.nodesWithin = nodes;   

            for j=1:length(nodes)
                tempNode = nodes{j};
                tempNode.withinObstacle = 1; %make the within obstacle flag true
            end 
        end
            
        
        function collisionEdges = findEdgesWithinObstacles(obj,F,G,tree,obstacles)
            
            collisionEdges = [];
            %for i = 1:obj.indexOfLast
            for i = 1:length(obstacles)

                if isempty(obstacles{i}) %just in case
                    continue
                end

                edges = findEdgesWithinEachObstacle(obj,F,G,tree,obstacles{i});
                collisionEdges = [collisionEdges edges'];
            end
        end
        

        function edges = findEdgesWithinEachObstacle(obj,F,G,tree,thisObstacle)
                
            if thisObstacle.status == 0 
                edges = [];
                return %if the obstacle is not active continue
            end

            centre  = thisObstacle.location;
            epsilon = thisObstacle.radius + obj.toleranceLimit; %extra-padding

            nodes = tree.kdFindWithinRangePayload(epsilon,centre);         
            motionEdgeIndices = [];      
            
            for i=1:length(nodes)
                thisNode = nodes{i};
                
                %fprintf('\n\nNode number: %d',i);
                %fprintf('\nNode index: %d',thisNode.index);
                
                % plot(thisNode.pose(1),thisNode.pose(2),'xg','MarkerSize',15,'LineWidth',3);
                % drawnow
                
                motionEdgeIndices = [motionEdgeIndices, obj.findEdgesInAugmentedGraph(G,thisNode,thisObstacle)];
            end

            
            if(isempty(motionEdgeIndices)) %if no edge is in collision, continue
                edges = [];
                return
            end
            
            %edgeIdices - all the edges that are in collision with this obstacle 
            %now we extract the edge pointers and save it
            edges(length(motionEdgeIndices),1) = edgeStruct();
            for j=1:length(motionEdgeIndices)
                tempEdgeIndex = motionEdgeIndices(j);
                tempEdge = G.graphEdges(tempEdgeIndex);

                tempEdge.cost = inf;
                tempEdge.withinObstacle = 1;
                
                edges(j) = tempEdge;
                
                %head.vertexData(2) corresponds to the funnel-edge
                %so should be tail.vertexData(2) (by construction)
                tempHead = G.graphVertices(tempEdge.parent);
                tempFunnel = F.funnelEdges(tempHead.vertexData(2));
                tempFunnel.cost = inf;
                tempFunnel.withinObstacle = 1;           
            end
            
            %store the edges data to this obstacle
            thisObstacle.edgesWithin = edges;
            %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%
            %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%
            %consider changing storing edges directly to edgeIndices
            %for saving space and improving runtime performance
            %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%
            %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%
        end
           

        function motionEdgesInCollision = findEdgesInAugmentedGraph(obj,G,thisNode,thisObstacle)
             
            s = 0;
            motionEdgesInCollision = NaN(length(thisNode.inletVertices) + length(thisNode.outletVertices),1);
            
            
            for i = 1:length(thisNode.inletVertices)

                thisInletVertex = G.graphVertices(thisNode.inletVertices(i));
                
                %each inlet vertex will have only one out-edge (motion-edge)
                tempEdgeIndex = thisInletVertex.outEdges;
                tempEdge = G.graphEdges(tempEdgeIndex);
                
                if isempty(tempEdge) %ideally, it shouldn't be empty
                    %disp('Empty edge encountered!')
                    continue
                end
                
                %one-more check -- in theory, not required (for debugging)
                if tempEdge.type == 0 %if continuity edge, 
                    disp('Bug: Continuity-edge checked!');
                    continue          %not relevant for collision checking
                end

                tempHead = G.graphVertices(tempEdge.parent);
                tempTail = G.graphVertices(tempEdge.child);

                %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%
                %Lane-based collision checking (very specific to the road-like workspace) 
                %if edge is not in the same lane (x-position) as the other car-obstacle
                %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%
                if tempHead.pose(1) == tempTail.pose(1) && tempHead.pose(1) ~= thisObstacle.location(1)
                    %plot([tempHead.pose(1); tempTail.pose(1)], [tempHead.pose(2); tempTail.pose(2)],'m','LineWidth',3);
                    %drawnow
                    continue
                end

                %Collision check to see whether edge is in collision
                if obj.edgeCollisionFreeWithThisObstacle(tempHead.pose,tempTail.pose,thisObstacle)
                    %fprintf('\n\n The edge that was skipped: %d',tempEdge.index);
                    %plot([tempHead.pose(1); tempTail.pose(1)], [tempHead.pose(2); tempTail.pose(2)],'g','LineWidth',3)
                    continue
                end
                        
                s = s+1;
                motionEdgesInCollision(s) = tempEdgeIndex;
                thisInletVertex.withinObstacle = 1; %not sure when this would become 0 again
                                                    %have to code it
            end
            
            for i = 1:length(thisNode.outletVertices)

                thisOutletVertex = G.graphVertices(thisNode.outletVertices(i));
                
                %each outlet vertex will have only one in-edge (motion-edge)
                tempEdgeIndex = thisOutletVertex.inEdges;
                tempEdge = G.graphEdges(tempEdgeIndex);
                
                if isempty(tempEdge) %ideally, it shouldn't be empty
                    %disp('Empty edge encountered!')
                    continue
                end
                
                %one-more check -- in theory, not required (for debugging)
                if tempEdge.type == 0 %if continuity edge, 
                    disp('Bug: Continuity-edge checked!');
                    continue          %not relevant for collision checking
                end

                tempHead = G.graphVertices(tempEdge.parent);
                tempTail = G.graphVertices(tempEdge.child);

                %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%
                %Lane-based collision checking (very specific to the road-like workspace) 
                %if edge is not in the same lane (x-position) as the other car-obstacle
                %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%
                if tempHead.pose(1) == tempTail.pose(1) && tempHead.pose(1) ~= thisObstacle.location(1)
                    %plot([tempHead.pose(1); tempTail.pose(1)], [tempHead.pose(2); tempTail.pose(2)],'m','LineWidth',3);
                    %drawnow
                    continue
                end

                %Collision check to see whether edge is in collision
                if obj.edgeCollisionFreeWithThisObstacle(tempHead.pose,tempTail.pose,thisObstacle)
                    %fprintf('\n\n The edge that was skipped: %d',tempEdge.index);
                    %plot([tempHead.pose(1); tempTail.pose(1)], [tempHead.pose(2); tempTail.pose(2)],'g','LineWidth',3)
                    continue
                end
                        
                s = s+1;
                motionEdgesInCollision(s) = tempEdgeIndex;
                thisOutletVertex.withinObstacle = 1; %not sure when this would become 0 again
                                                     %have to code it!
            end
            
            motionEdgesInCollision = unique(motionEdgesInCollision(1:s))'; %to remove any possible duplicates
        end
        
        
        function modifiedEdges = getModifiedEdges(obj,F,C,G,tree,obstacles,type)
            
            if isempty(obstacles)
                modifiedEdges = [];
                return
            end

            if nargin < 7
                type = 'addition';
            end
            
            if strcmpi(type,'addition') %this list comprises of added obstacles
                %so determine the edges in collision first
                obj.findNodesWithinObstacles(C,tree,obstacles);
                modifiedEdges = obj.findEdgesWithinObstacles(F,G,tree,obstacles);
                modifiedEdges = unique(modifiedEdges); %removing duplicates
                return
            end
            
            %this list would comprise of deleted obstacles, so just return
            %the edges pre-stored within the removed obstacles
            modifiedEdges = [];
            for i=1:length(obstacles)
                modifiedEdges = [modifiedEdges obstacles{i}.edgesWithin'];
            end
            modifiedEdges = unique(modifiedEdges); %removing duplicates

        end
        
        %plotting functions
        
        %function to draw all obstacles
        function drawAllObstacles(obj)            
            
            %if you want to plot the bounding circle-obstacles
            % for i=1:obj.indexOfLast
            %     thisObstacle = obj.obstacles{i};
            %     if(thisObstacle.status == 0)
            %         obj.drawDeletedObstacle(thisObstacle);
            %     else %plot inactive obstacles with dashed circles
            %         obj.drawActiveObstacle(thisObstacle);
            %     end
            % end

            %if you want to plot recangular car-obstacles
            for i=1:3:obj.indexOfLast
                thisRectangleObstacle = obj.boundingRectangles{ceil(i/3)};
                if(obj.obstacles{i}.status == 1 || obj.obstacles{i+1}.status == 1 || obj.obstacles{i+2}.status == 1)
                    thisRectangleObstacle.drawSensedRectangle()
                else %plot inactive obstacles with dashed lines
                    thisRectangleObstacle.drawUnSensedRectangle()
                end
            end
        end
        
        %Function to draw shaded circular obstacles
        function drawActiveObstacle(obj,obstacle)
            
            center = obstacle.location;
            radius = obstacle.radius;

            th = 0:pi/50:2*pi;
            xunit = radius * cos(th) + center(1);
            yunit = radius * sin(th) + center(2);
            
            %plot(xunit, yunit,'k','LineWidth', 1.1);
            fill(xunit,yunit,[0.3 0.3 0.3])
            plot(center(1),center(2),'xw')
        end
        
        %function to draw a deleted obstacle with dashed line
        function drawDeletedObstacle(obj,obstacle)
            
            color = [1 1 1];
            c = obstacle.location;
            r = obstacle.radius;
            
            th = 0:pi/50:2*pi;
            xunit = r * cos(th) + c(1);
            yunit = r * sin(th) + c(2);
            fill(xunit,yunit,[1 1 1], 'EdgeColor', 'none'); %fill with white
            
            fill(xunit,yunit,[1 1 1], 'EdgeColor', 'none','FaceAlpha',0.05); %fill with white
            %Plot just the outline
            plot(xunit, yunit,'--k','LineWidth',1.1);
        end
        
        %Function to draw circle representing sensor radius
        function drawSensorRadius(obj,c)
            r = obj.sensorRadius;
            hold on
            th = 0:pi/50:2*pi;
            xunit = r * cos(th) + c(1);
            yunit = r * sin(th) + c(2);
            plot(c(1),c(2),'xk')
            plot(xunit, yunit,'-.k');
        end

        %Collision checking functions

        %Checking whether point lies within a circle
        
        function check = vertexCollisionFree(obj,v)

            check = 1;
            
            for i = 1:obj.indexOfLast
                
                thisObstacle = obj.obstacles{i};
                if(thisObstacle.status == 0) %if inactive continue
                    continue
                end
                %Accessing the centre and radius from the obstacles file
                centre = thisObstacle.location;
                radius = thisObstacle.radius;

                %Checking if node v is inside the obstacle            
                if (euclidianDist(obj,centre,v) < radius) 
                    check = 0;
                    return
                end
            end
        end

        %returns 1 if edge is free of collision else return 0
        %edge - (v,w) v <-- parent w <-- child
        %Collision check of a line and circle
        function check = edgeCollisionFree(obj,v,w)
            
            check = 1;
            %v = edgeHead.pose;
            %w = edgeTail.pose;
            if(~vertexCollisionFree(obj,w) || ~vertexCollisionFree(obj,v))
                check = 0;
                return;
            end

            for i = 1:obj.indexOfLast
                
                thisObstacle = obj.obstacles{i};
                if(thisObstacle.status == 0) %if inactive continue
                    continue
                end
                
                %Accessing the centre and radius from the obstacles file
                centre = thisObstacle.location;
                %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%
                %radius = thisObstacle.radius + obj.toleranceLimit; %new addition -- extra padding
                radius = thisObstacle.radius;
                %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%

                %Checking if the edge (v,w) intersects the circle
                
                angleSubtend = ((centre(1)-v(1))*(w(1)-v(1)) + (centre(2)-v(2))*(w(2)-v(2)))/(euclidianDist(obj,v,w)^2);

                xProjection = v(1) + angleSubtend*(w(1)-v(1));
                yProjection = v(2) + angleSubtend*(w(2)-v(2));

                closestPoint = [xProjection, yProjection];

                %If the closest point lies within the edge and as well as at a
                %distance less than the radius, it implies collision
                %if((euclidianDist(obj,closestPoint,centre) < radius))
                if(liesInBetween(obj,v,w,closestPoint) && (euclidianDist(obj,closestPoint,centre) < radius))
                    check = 0;
                    return
                end
            end
        end
        
        %returns 1 if edge is free of collision else return 0
        %edge - (v,w) v <-- parent w <-- child
        %Collision check of a line and circle
        function check = edgeCollisionFreeWithThisObstacle(obj,v,w,thisObstacle)
            
            check = 1;
            %v = edgeHead.pose;
            %w = edgeTail.pose;
            if(~vertexCollisionFree(obj,w) || ~vertexCollisionFree(obj,v))
                check = 0;
                return;
            end

            if(thisObstacle.status == 0) %if inactive continue
                return
            end

            %Accessing the centre and radius from the obstacles file
            centre = thisObstacle.location;
            radius = thisObstacle.radius + obj.toleranceLimit; %new addition -- extra padding

            %Checking if the edge (v,w) intersects the circle

            angleSubtend = ((centre(1)-v(1))*(w(1)-v(1)) + (centre(2)-v(2))*(w(2)-v(2)))/(euclidianDist(obj,v,w)^2);

            xProjection = v(1) + angleSubtend*(w(1)-v(1));
            yProjection = v(2) + angleSubtend*(w(2)-v(2));

            closestPoint = [xProjection, yProjection];

            %If the closest point lies within the edge and as well as at a
            %distance less than the radius, it implies collision
            %if((euclidianDist(obj,closestPoint,centre) < radius))
            if(liesInBetween(obj,v,w,closestPoint) && (euclidianDist(obj,closestPoint,centre) < radius))
                check = 0;
                return
            end
        end
        
        function success = funnelCollisionFree(obj,funnel)
            success = 1;
            
            traj = funnel.trajectory_workSpace;
            
            initialConfig  = traj(:,1);
            finalConfig = traj(:,end);
            midConfig = (initialConfig+finalConfig)/2; %computing the approx centre of the trajectory
            boundingCircleRadius = 1*euclidianDist(obj,initialConfig,finalConfig)/2; %coefficient: scaling for safety          
            
            for i = 1:obj.indexOfLast
               
                thisObstacle = obj.obstacles{i};
                if(thisObstacle.status == 0) %if inactive continue
                    continue
                end

                if(boundingCircleCheck(obj,midConfig,boundingCircleRadius,thisObstacle)) %if the funnel is sufficiently far off
                    continue                                         %from the obstacle return with 1   
                end

                if(~funnelCircleCollision(obj,funnel,thisObstacle))
                    success=0;
                    return
                end
            end
        end
        
        function success = funnelCollisionFreeWithThisObstacle(obj,funnel,thisObstacle)
            success = 1;
            
            traj = funnel.trajectory_workSpace;
            
            initialConfig  = traj(:,1);
            finalConfig = traj(:,end);
            midConfig = (initialConfig+finalConfig)/2; %computing the approx centre of the trajectory
            boundingCircleRadius = 1*euclidianDist(obj,initialConfig,finalConfig)/2;
            
            if(thisObstacle.status == 0) %if inactive continue
                return
            end

            if(boundingCircleCheck(obj,midConfig,boundingCircleRadius,thisObstacle)) %if the funnel is sufficiently far off
                return                                         %from the obstacle return with 1   
            end

            if(~funnelCircleCollision(obj,funnel,thisObstacle))
                success=0;
                return
            end
        end
        
    end
    

    %if we have to declare functions that are required to be 
    %private in scope. Can be accessed from only within this class
    methods (Access = private)
        %distance function
        function dist = euclidianDist(obj,v,w)
            dist = sqrt(sum((v - w).^2,2));
        end
        
        %if a point lies within a linesegment VW returns 1
        function check = liesInBetween(obj,v,w,P)
            check = 1;
            if (P - v)*(P - w)' > 0
                check = 0;
            end
        end
        
        %rough check by drawing a bounding circle on the ellipse
        function success = firstPass(obj,c,r,a)
            if norm(c)>(a+r)
                success=1;
            else
                success=0;
            end
        end

        %Function to generate Van Der Corput sequence %N - array length, b - base
        function s = vdcorput(obj,N,b)  %output - N+1 (starting with ZERO)
            s = zeros(N,1);
            for i = 1:N
                a = basexpflip(obj,i,b);
                g = b.^(1:length(a));
                s(i) = sum(a./g);
            end    
        end

        %Reversed base-b expansion of positive integer k
        function a = basexpflip(obj,k,b) 
            j = fix(log(k)/log(b)) + 1;
            a = zeros(1,j);
            q = b^(j-1);
            for i = 1:j
               a(i) = floor(k/q);
               k = k - q*a(i);
               q = q/b;
            end
            a = fliplr(a);
        end
        
        %Function to perform a course-check whether the point is not in the
        %bounding circle of the funnel, returns 1 if point is outside the circle
        function pass = boundingCircleCheck(obj,midState,funnelRadius,obstacle)
            pass = 0;
            center = obstacle.location;
            radius = obstacle.radius; 
            if(euclidianDist(obj,midState,center)>funnelRadius+radius)
                pass = 1;
                return
            end
        end
        
        %checks collision b/w funnel and each circular obstacle
        function success = funnelCircleCollision(obj,funnel,obstacle)
            
            success = 1;
            N = length(funnel.time);
            vanDerSequence = ceil(vdcorput(obj,N,2)*N);

            for k = 1:N
                index = vanDerSequence(k);
                x_c = funnel.trajectory_workSpace(:,index);
                M = funnel.invariantSet_workSpace(:,:,index);

                if(~ellipseCircleCollisionFree(obj,x_c,M,obstacle))
                    success = 0;
                    return
                end
            end
        end

        %Collision-check between an ellipse and list of circular obtacles
        function success = ellipseCircleCollisionFree(obj,ellipseCentre,M,obstacle)

            success = 1;
            %for algebraic analaysis
            [~, D, V] = svd(inv(M));

            a = sqrt(max(diag(D))); b = sqrt(min(diag(D)));
            c = sqrt(a^2 - b^2); %Focal length

            pointA = [c 0]'; pointB = [-c 0]';

            %Transforming along principal coordinates
            circleCentre = obstacle.location';
            circleRadius = obstacle.radius;
            pointC = V*(circleCentre - ellipseCentre);

            if firstPass(obj,pointC,circleRadius,a)
                return
            end

            %Detecting the point P of intersection should come over here
            pointP = pointOfIntersection(obj,pointA, pointC, circleRadius);
            dist1 = norm(pointP - pointA) + norm(pointP - pointB);

            pointP = pointOfIntersection(obj,pointB, pointC, circleRadius);
            dist2 = norm(pointP - pointA) + norm(pointP - pointB);

            if(dist1 < 2*a || dist2 < 2*a)
                success = 0;
                return
            end
        end

        %Function to determine the point of intersection b/w
        %a ray from a point A to the centre and the circle
        function P = pointOfIntersection(obj,point,center,radius)
            phi = atan2(point(2)-center(2), point(1)-center(1));
            x = center(1) + radius * cos(phi);
            y = center(2) + radius * sin(phi);
            P = [x;y];
        end
    end
end