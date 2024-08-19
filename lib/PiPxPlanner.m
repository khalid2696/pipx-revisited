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
        envLB
        envUB
    end %end of properties

    methods
        function obj = PiPxPlanner(envLB,envUB) %constructor class
            
            obj.envLB = envLB;
            obj.envUB = envUB;

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

        function flag = generateFunnelRRG(obj,F,C,G,O,T,startFound,robotMove,epsilon)

            newNodePose = C.expandSearchGraph(T,O,startFound,robotMove,epsilon);
            %plot(newNodePose(1),newNodePose(2), 'xb','MarkerSize',7,'LineWidth',1.4)
            
            if(~O.vertexCollisionFree(newNodePose) || F.inFunnel(newNodePose))
                flag = 1; return
            end
            
            %Find the neighbors within an r-Ball
            [neighbors, flag] = obj.findNeighborsInRBall(T,newNodePose);
         
            if flag %if no neighbors found, continue with the next sampling
                return
            end
        
            thisNode = nodeStruct(C.numNodes+1,newNodePose);
            
            %add the the new sampled node to existing funnel-network 
            flag = F.constructFunnelNetwork(T,C,O,thisNode,neighbors);
            
            if flag %if no new edges were added, continue with the next sampling
                return
            end
            
            G.constructAugmentedGraph(F,C,thisNode);
        end
        
        function flag = addStartNodeToFunnelRRG(obj,F,C,G,O,T,startPose)
        
            nearestNeighbor = C.graphNodes(end).pose; %using the neighbors of the previous node
            [neighbors, flag] = obj.findNeighborsInRBall(T,nearestNeighbor);
        
            if flag
                return
            end
            
            startNode = nodeStruct(C.numNodes+1,startPose);
            flag = F.constructFunnelNetwork(T,C,O,startNode,neighbors);
            
            if flag
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
            
            %move the robot to its parent node
            parentIndex = C.startNode.parent;
            C.startNode = C.graphNodes(parentIndex);
            F.startNode = C.startNode;
            C.currentRobotNode = C.startNode;
        end
        
        %-------------------------------------------------------------------------%
        %Environment dynamicity (as sensed by the robot) related function
        function makeDynamicChangesToGraph(obj,F,C,G,Q,O,T)   
        
            exploredObstacles = O.senseObstacles(C.currentRobotNode.pose); %sense from the middle
            modifiedEdges = O.getModifiedEdges(F,C,G,T,exploredObstacles);
            
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
                neighbors = nodeStruct();
                flag = 1;
                return
            end
        end
        
        %Shrinking r-Ball
        function r = rBall(obj,iteration)
            
            %Shrinking rate from RRT* paper
            epsilon = 5.5; r0 = 800; d = 2; iteration = iteration+1;
            r = min(r0*(log(iteration)/(iteration))^1/d,epsilon);
        end
        
        %-------------------------------------------------------------------------%
        %Plotting functions
        function setupPlot(obj)
            figure
            clf
            axis equal
            xlim([obj.envLB obj.envUB])
            ylim([obj.envLB obj.envUB])
            hold on
            rectangle('Position',[obj.envLB obj.envLB obj.envUB obj.envUB])
        end
        
        %-------------------------------------------------------------------------%
        %Saving data functions
        function fileCount = saveData(obj,F,C,O,dir,fileCount)
            nodes = C.graphNodes;
            edges = C.graphEdges;
            funnels = F.funnelEdges;  
            obstacles = O.obstacles;
            robotNode = C.startNode;
            save([dir 'iteration_' num2str(fileCount) '.mat'],'nodes','edges','funnels','obstacles','robotNode');
            
            fileCount = fileCount+1;
        end
    
    end %end of methods
    
end %end of class defintion