% The MIT License (MIT)
%
% Copyright 2024 Mohamed Khalid M Jaffar, University of Maryland
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

%Defining a class to store the functions related to configuration space
classdef PiPxPlanner < handle
    properties
        envLB_x
        envUB_x
        envLB_y
        envUB_y
        extendDistance
        resolution
        CspaceDimensionality
        initial_rBall_radius
        drawFlag
    end %end of properties

    methods
        function obj = PiPxPlanner(envLB_x,envUB_x,envLB_y,envUB_y,extendDistance,resolution,drawFlag) %constructor class
            
            obj.envLB_x = envLB_x - 0.5; obj.envUB_x = envUB_x + 0.5; %adding/subtracting 0.5 to better visualize road-lanes
            obj.envLB_y = envLB_y; obj.envUB_y = envUB_y;
            obj.extendDistance = extendDistance;
            obj.resolution = resolution;
            obj.CspaceDimensionality = 2; %x-theta planning
            obj.initial_rBall_radius = 50; %used for the shrinking rBall radius compute

            if nargin < 5
                obj.drawFlag = 0;
            else
                obj.drawFlag = drawFlag;
            end

            if(nargin == 0)
            
                % obj.numNodes = 0;
                % obj.numEdges = 0;
                % 
                % obj.graphNodes = nodeStruct();
                % obj.graphEdges = edgeStruct();
                % 
                % obj.startNode = []; %will be updated in runtime
                % obj.goalNode  = []; %will be updated in runtime
                % 
                % obj.previousRobotNode = []; %will be updated during robot motion
                % obj.currentRobotNode = []; %will be updated during robot motion
                return
            end
            
            % obj.graphNodes = nodes;
            % obj.graphEdges = edges;
            % 
            % obj.numNodes = length(nodes);
            % obj.numEdges = length(edges);
            % 
            % obj.startNode = []; %will be updated in runtime
            % obj.goalNode  = []; %will be updated in runtime
        end

        function flag = generateFunnelRRG(obj,F,C,G,W,T,startFound,robotMove)
            
            %epsilon = obj.extendDistance*obj.CspaceDimensionality^(1/obj.CspaceDimensionality); %L2-norm of the extend distance (xy-planning)
            epsilon = obj.extendDistance; %using a conservative epsilon (extendDistance) for expanding the searchFunnel
                                          %note that although, in principle, we should be able to
                                          %use the previous line epsilon (upper bound value) as well
            newNodePose = C.expandSearchGraph(T,W,startFound,robotMove,epsilon,F.resolution);
            
            if(~W.vertexCollisionFree(newNodePose) || F.inAnyInlets(newNodePose))
                flag = 1; return
            end
            
            %Find the neighbors within an r-Ball
            [potentialNeighbors, flag] = obj.findNeighborsInRBall(T,newNodePose);
         
            if numel(potentialNeighbors) == 0 %if no neighbors found, continue with the next sampling
                return
            end
            
            thisNode = nodeStruct(C.numNodes+1,newNodePose);

            %add the the new sampled node to existing funnel-network 
            flag = F.constructFunnelNetwork(T,C,W,thisNode,potentialNeighbors);
            
            if flag %if no new edges were added, continue with the next sampling
                return
            end
            
            G.constructAugmentedGraph(F,C,thisNode);

            %plot(thisNode.pose(1),thisNode.pose(2), 'xy','MarkerSize',7,'LineWidth',1.4)
            %drawnow
        end
        
        function flag = addStartNodeToFunnelRRG(obj,F,C,G,W,T,startPose)
        
            %nearestNeighbor = C.graphNodes(end).pose; %using the neighbors of the previous node
            %[neighbors, flag] = obj.findNeighborsInRBall(T,nearestNeighbor);
            
            [potentialNeighbors, flag] = obj.findNeighborsInRBall(T,startPose);

            if numel(potentialNeighbors) == 0 %if no neighbors found, exit
                return
            end
            
            startNode = nodeStruct(C.numNodes+1,startPose);
            flag = F.constructFunnelNetwork(T,C,W,startNode,potentialNeighbors);
            
            if flag %if no funnel-edges were added, exit
                return
            end
            
            G.constructAugmentedGraph(F,C,startNode);
            C.startNode = startNode; F.startNode = startNode;
        end
        
        function status = moveRobot(obj,F,C,G,Q)
            
            if C.goalCheck(C.startNode.pose)
                status = 1; %if already at goal region break out
                return
            end
        
            %save robot's previous node information
            G.updatekm(C); %needs to be updated only when robot moves
            C.previousRobotNode = C.startNode;   
            
            %computes the solution path
            status = C.findBestInletAtStartNode(G,F,Q);
            
            if ~status
                return
            end

            C.findParentInletsAtEachNode(G);
            F.constructShortestFunnelPath(G); 
            
            if obj.drawFlag
                traversingFunnelEdge = F.funnelEdges(C.currentRobotNode.parentFunnelEdge);
                F.drawFunnel(traversingFunnelEdge,2);
            end
            
            %move the robot to its parent node
            parentIndex = C.startNode.parent;
            C.startNode = C.graphNodes(parentIndex);
            F.startNode = C.startNode;
            C.currentRobotNode = C.startNode;
        end
        
        %-------------------------------------------------------------------------%
        %Environment dynamicity (as sensed by the robot) related function
        function makeDynamicChangesToGraph(obj,F,C,G,Q,W,T,varargin)   
        
            if strcmpi(W.mode, 'sensing') 
                exploredObstacles = W.senseObstacles(C.currentRobotNode.pose); %sense from the middle
                modifiedEdges = W.getModifiedEdges(F,C,G,T,exploredObstacles);
            elseif strcmpi(W.mode, 'dynamic') 
               
                if W.numObstacles == 0 || W.dynamicity == 0
                    modifiedEdges = [];
                else
                    %deletion
                    [deletedObstacles, deletedBoundingObstacles] = W.removeRandomObstacles(F,G);
                    freedUpEdges = W.getModifiedEdges(F,C,G,T,deletedObstacles,'deletion');
                    if obj.drawFlag
                        W.drawAllObstacles();
                        % for j = 1:numel(deletedObstacles)
                        %     thisObstacle = deletedObstacles{j};
                        %     W.drawDeletedObstacle(thisObstacle);
                        % end
                    end
                    %addition
                    addedObstacles = W.addShiftedObstacles(deletedBoundingObstacles,C.currentRobotNode.pose,C.goalNode.pose);
                    newCollisionEdges = W.getModifiedEdges(F,C,G,T,addedObstacles,'addition');
                    modifiedEdges = [freedUpEdges, newCollisionEdges];
                end
            else
                modifiedEdges = [];
            end

            modifiedEdges = unique(modifiedEdges); %removing duplicates for computation speed-up

            if (isempty(modifiedEdges))
                return
            end
            
            for i=1:length(modifiedEdges)
                vertexIndex = modifiedEdges(i).parent;
                vertex = G.graphVertices(vertexIndex);
                G.updateVertex(Q,vertex);
            end
        end
        
        %Determine the neighbors in the rBall
        function [neighbors, flag] = findNeighborsInRBall(obj,T,newNodePose)
            
            %Determine the radius of the r ball
            r = obj.rBall(T.treeSize); %T.treesize --> number of configurations sampled
            
            flag = 0;
            
            neighbors = T.kdFindWithinRangePayload(r,newNodePose);
            if isempty(neighbors) %if no parent can be found within the radius ball continue
                neighbors = [];
                flag = 1;
                return
            end
        end
        
        %Shrinking r-Ball
        function r = rBall(obj,iteration)
            
            %Shrinking rate from RRT* paper 
            r0 = obj.initial_rBall_radius; iteration = iteration+1;
            %epsilon = obj.extendDistance * obj.CspaceDimensionality^(1/obj.CspaceDimensionality); %L_infinity-norm to L2-norm conversion
            epsilon = obj.extendDistance; %L_infinity_norm

            r = min(r0*(log(iteration)/(iteration))^(1/obj.CspaceDimensionality), epsilon); %upper saturation
            r = max(r, max(obj.resolution));  %lower saturation
            
            %rBall radius is saturated by max extend distance (UB) and
            %resolution of the funnelLibrary (LB)
        end
        
        %-------------------------------------------------------------------------%
        %Plotting functions
        function setupPlot(obj)
            figure; clf;
            axis equal
            %daspect([2 1 1])
            xlim([obj.envLB_x obj.envUB_x])
            ylim([obj.envLB_y obj.envUB_y])
            hold on
            rectangle('Position',[obj.envLB_x, obj.envLB_y, obj.envUB_x - obj.envLB_x, obj.envUB_y - obj.envLB_y]) %lower-left corner, width, height
            xlabel('x');
            ylabel('\theta');
        end
        
        %-------------------------------------------------------------------------%
        %Saving data functions
        function fileCount = saveData(obj,F,C,W,dir,fileCount)
            nodes = C.graphNodes;
            edges = C.graphEdges;
            funnels = F.funnelEdges;  
            obstacles = W.obstacles;
            robotNode = C.startNode;
            save([dir 'iteration_' num2str(fileCount) '.mat'],'nodes','edges','funnels','obstacles','robotNode');
            
            fileCount = fileCount+1;
        end
    
    end %end of methods
    
end %end of class defintion