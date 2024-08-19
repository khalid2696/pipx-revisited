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

%Defining a class to store the functions related to configuration space
classdef configurationSpace < handle
    properties
        
        numNodes 
        numEdges
        
        graphNodes
        graphEdges
        
        startNode
        goalNode
        
        %for robot movement
        previousRobotNode
        currentRobotNode
    end
    methods
        function obj = configurationSpace(nodes,edges)
            
            if(nargin == 0)
            
                obj.numNodes = 0;
                obj.numEdges = 0;
                
                obj.graphNodes = nodeStruct();
                obj.graphEdges = edgeStruct();
                
                obj.startNode = []; %will be updated in runtime
                obj.goalNode  = []; %will be updated in runtime
                
                obj.previousRobotNode = []; %will be updated during robot motion
                obj.currentRobotNode = []; %will be updated during robot motion
                return
            end
            
            obj.graphNodes = nodes;
            obj.graphEdges = edges;
            
            obj.numNodes = length(nodes);
            obj.numEdges = length(edges);
            
            obj.startNode = []; %will be updated in runtime
            obj.goalNode  = []; %will be updated in runtime
        end
        
        function obj = addNode(obj,node)
                obj.numNodes = obj.numNodes + 1;
                obj.graphNodes(obj.numNodes) = node;
                if isnan(node.index)
                    node.index = obj.numNodes;
                end
        end
        
        function obj = addEdge(obj,edge)
                obj.numEdges = obj.numEdges + 1;
                obj.graphEdges(obj.numEdges) = edge;
                if isnan(edge.index)
                    edge.index = obj.numEdges;
                end
        end
        
        %-------------------------------------------------------------------------%
        %function to sample configurations
        
        function sample = sampleNode(obj,O,startFound,robotMove)

            start = obj.startNode.pose;
            if(robotMove)
                prob = rand();
                if prob < 0.7 %0.9
                    r = O.sensorRadius*rand() + 3; %epsilon away
                else
                    %r = 2*O.sensorRadius*rand() + 2; %1.5
                    xSample = (O.envUB - O.envLB)*rand() + O.envLB;
                    ySample = (O.envUB - O.envLB)*rand() + O.envLB;
                    sample = [xSample ySample];
                    return
                end

                theta  = 2*pi*rand();
                sample = [start(1)+r*cos(theta) start(2)+r*sin(theta)];
                %plot(sample(1),sample(2),'xb');
                return
            end

            if(~startFound)
                bias = 0.9; %0.95
            else
                bias = 1;
            end

            prob = rand();
            if prob < bias
                %random sampling of nodes
                xSample = (O.envUB - O.envLB)*rand() + O.envLB;
                ySample = (O.envUB - O.envLB)*rand() + O.envLB;
                sample = [xSample ySample];
            else
                sample = start;
            end

        end
        
        %function to extrapolate to a new configuration from the nearest neighbor in
        %the existing RRG
        function newNodePose = expandSearchGraph(obj,T,O,startFound,robotMove,epsilon)
            
            %sampling a point at random
            sampledPoint = obj.sampleNode(O,startFound,robotMove);
            %plot(sampledPoint(1),sampledPoint(2), 'xy','MarkerSize',7,'LineWidth',1.4)

            nearestNode = T.kdFindNearestPayload(sampledPoint);
              
            distance = obj.computeEuclidianDist(nearestNode.pose,sampledPoint);
            if(distance < epsilon)
                newNodePose = sampledPoint;
            else
                t = epsilon/distance;
                newNodePose(1) = (1-t)*nearestNode.pose(1) + t*sampledPoint(1);
                newNodePose(2) = (1-t)*nearestNode.pose(2) + t*sampledPoint(2);
            end
        end

        %-------------------------------------------------------------------------%
        %defining all cost functions at one place 
        function d = computeEuclidianDist(obj,v,u)
            d = sqrt((v(1)-u(1))^2 + (v(2)-u(2))^2);
        end

        %cost function definition
        function cost = computeCostFn(obj,v,u)
            cost = obj.computeEuclidianDist(v,u);
        end
        
        %Goal checking
        function success = goalCheck(obj,inputPose)
            success = norm(inputPose-obj.goalNode.pose(1:2)) < 0.5; %tolerance limit is 0.5
        end
        
        %-------------------------------------------------------------------------%
        %below two routines are *necessary* during robot motion and also
        %needs to be run for visualisation of robot's solution funnel-path
        function status = findBestInletAtStartNode(obj,G,F,Q)

            prevBest = inf;
            bestStartVertexIndex = NaN;
            status = 0;
            
            G.startVertex = G.graphVertices(obj.startNode.inletVertices(1));
            %G.computeShortestPath(Q);

            for i=1:length(obj.startNode.inletVertices)
                tempStartVertex = G.graphVertices(obj.startNode.inletVertices(i));
                
                G.startVertex = tempStartVertex;    
                G.computeShortestPath(Q);
                
                nextFunnel = F.funnelEdges(tempStartVertex.vertexData(2));
                if ~F.inFunnelInlet(nextFunnel,G.startVertex.pose)
                    continue %we can't take this inlet, because it doesn't contain start pose
                end
                                
                if G.startVertex.cost < prevBest
                    prevBest = G.startVertex.cost;
                    bestStartVertexIndex = G.startVertex.index;
                end
            end
              
            if ~isinf(prevBest)
                G.startVertex = G.graphVertices(bestStartVertexIndex);
                G.computeShortestPath(Q);
                status = 1;
            end
            
        end
        
        function status = findParentInletsAtEachNode(obj,G)
            
            for i=2:obj.numNodes %goal node doesn't need to have an inlet assigned
                                 %because 'goal region' itself is a (universal) inlet
                                 %that's why the loop starts from 2
                thisNode = obj.graphNodes(i);

                if thisNode.withinObstacle %if it's in collision
                    continue
                end
                
                minCost = inf;
                bestOutlet = NaN;
                for j=1:length(thisNode.outletVertices)
                    tempVertex = G.graphVertices(thisNode.outletVertices(j));
                    if tempVertex.cost < minCost && ~tempVertex.inHeap
                        minCost = tempVertex.cost;
                        bestOutlet = tempVertex.index;
                    end
                end
                
                %if no best outlets found
                if isnan(bestOutlet) || minCost > 1.1*G.startVertex.cost
                    thisNode.parent = NaN;
                    thisNode.parentFunnelEdge = NaN;
                    
                    thisNode.bestInlet = NaN;
                    thisNode.cost = inf; %outlet is the same cost as inlet (because zero-cost edge)
                    
                    continue
                end
                
                if ~isnan(bestOutlet)
                    
                    %accessing the parent inlet vertex of the best outlet vertex
                    parentInlet = G.graphVertices(bestOutlet).parent;
                    
                    %Inlet vertices will have one and only one outNeighbor 
                    %(by construction) -- This would be the best parent node in the 
                    %funnel network  
                    tempParentOutletIndex = G.graphVertices(parentInlet).outNeighbors;
                    tempParentOutlet = G.graphVertices(tempParentOutletIndex);

                    %vertex data -- [node,funnel]
                    tempVertexData = tempParentOutlet.vertexData;
                    thisParent = tempVertexData(1);
                    
                    thisNode.parent = thisParent;
                    thisNode.parentFunnelEdge = tempVertexData(2);
                    
                    thisNode.bestInlet = parentInlet;
                    thisNode.cost = minCost; %outlet is the same cost as inlet (because zero-cost edge) 
                end
            end

            status = ~isinf(obj.startNode.cost); %if infinite cost, no path exists CURRENTLY

            %if ~isnan(obj.startNode.bestInlet)
            %    G.startVertex = G.graphVertices(obj.startNode.bestInlet);
            %end
        end
        %-------------------------------------------------------------------------%
        
        
        function drawSearchGraph(obj)

            nodes = NaN(obj.numNodes,2);
            for i=1:obj.numNodes
                nodes(i,:) = obj.graphNodes(i).pose;
            end
            
            % a bit of data processing for faster plotting
            edges = NaN(3*obj.numEdges,2);
            for i=3:3:3*obj.numEdges
                tempEdge = obj.graphEdges(i/3);
                edges(i-2,:) = obj.graphNodes(tempEdge.child).pose; % edge starts
                edges(i-1,:) = obj.graphNodes(tempEdge.parent).pose; % edge ends
            end 
            
            plot(edges(:,1),edges(:,2),'Color',[0.3010, 0.7450, 0.9330],'LineWidth',3); % [0, 0.4470, 0.7410]
            plot(nodes(:,1),nodes(:,2), 'o','Color',[0, 0.4470, 0.7410]); %[0, 0.4470, 0.7410]
            
            plot(obj.goalNode.pose(1), obj.goalNode.pose(2), 'xr', 'MarkerSize', 8, 'LineWidth', 3.5)
            plot(obj.startNode.pose(1), obj.startNode.pose(2), 'sg', 'MarkerSize', 8, 'LineWidth', 3.5)  
        end
        
        function drawSearchTree(obj)
            edges = NaN(3*obj.numNodes,2);
            for i=6:3:3*obj.numNodes
                tempNode = obj.graphNodes(i/3); %starting from 2, because 1 is goal node which doesn't have a parent 
                if(isnan(tempNode.parent))
                    continue
                end
                edges(i-2,:) = obj.graphNodes(tempNode.parent).pose; % edge starts
                edges(i-1,:) = tempNode.pose; % edge ends
            end 

            %plot(edges(:,1),edges(:,2),'Color','y','LineWidth',2.7); %[0, 0.4470, 0.7410], [0.3010, 0.7450, 0.9330] 
            plot(edges(:,1),edges(:,2),'Color',[0, 0.4470, 0.7410],'LineWidth',2.5); %1.9 %[0.3010, 0.7450, 0.9330] 
              
            plot(obj.goalNode.pose(1), obj.goalNode.pose(2), 'xr', 'MarkerSize', 8, 'LineWidth', 3.5)
            plot(obj.startNode.pose(1), obj.startNode.pose(2), 'sg', 'MarkerSize', 8, 'LineWidth', 3.5)  
        end
        
        function drawPathToGoal(obj)
    
            %drawSearchTree(obj);
            start = obj.startNode.pose;
            goal = obj.goalNode.pose;
            delta = 0.1;
            
            if obj.startNode.index == obj.goalNode.index %if already at start
                return
            end
                
            j = 1;
            %Constructing the goal branch by backtracking through parent pointers
            temp = obj.startNode;
            while 1

                if (isnan(temp.parent))
                %if (isempty(temp) || isnan(temp.parent))
                    disp('There does not exist a feasible path currently!');
                    return
                    %break
                end

                goalBranch(j,:) = temp.pose;  
                temp = obj.graphNodes(temp.parent);
                if abs(temp.pose - goal) < delta
                    goalBranch(j+1,:)=goal;
                    break
                end
                j = j+1;
            end

            plot(start(1),start(2), 'sg','MarkerSize', 6,'LineWidth',2);
            plot(goalBranch(:,1),goalBranch(:,2),'-.g','LineWidth', 5); %2.5
            plot(goal(1),goal(2),'xr','MarkerSize', 6,'LineWidth',2);
        end
    end
end