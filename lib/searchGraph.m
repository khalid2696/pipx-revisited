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

%Defining a class to store the graph data structure representing the funnel-network
classdef searchGraph < handle
    properties
        
        numVertices 
        numEdges
        
        graphVertices
        graphEdges
        
        startVertex
        goalVertex
        
        %Graph-search related fields
        km
        previousPose
    end
    
    methods
        function obj = searchGraph(nodes,edges)
            
            if(nargin == 0)
            
                obj.numVertices = 0;
                obj.numEdges = 0;
                
                obj.graphVertices = augmentedVertexStruct();
                obj.graphEdges = edgeStruct();
                
                obj.startVertex = []; %will be updated in runtime
                obj.goalVertex  = []; %will be updated in runtime
                return
            end
            
            obj.graphVertices = nodes;
            obj.graphEdges = edges;
            
            obj.numVertices = length(nodes);
            obj.numEdges = length(edges);
            
            obj.startVertex = []; %will be updated in runtime
            obj.goalVertex  = []; %will be updated in runtime
        end
        
        %-------------------------------------------------------------------------%
        %constructing the graph iteratively from the RRG
        
        function obj = constructAugmentedGraph(obj,F,C,thisNode)

            for i=1:length(thisNode.inEdges) %outlet - newNode, inlet - neighborNode
                thisInEdge = C.graphEdges(thisNode.inEdges(i));
                thisInNeighbor = C.graphNodes(thisNode.inNeighbors(i)); %or C.graphNodes(C.graphEdges(thisInEdge).parent);

                %vertices
                inletVertex = augmentedVertexStruct(nan,thisInNeighbor.index,thisInEdge.index);
                inletVertex.type = 'inlet'; inletVertex.pose = thisInNeighbor.pose;
                outletVertex = augmentedVertexStruct(nan,thisNode.index,thisInEdge.index);
                outletVertex.type = 'outlet'; outletVertex.pose = thisNode.pose;

                obj.addVertex(inletVertex); %NaN index -- will get auto-assigned when added to the graph
                obj.addVertex(outletVertex);

                inletVertex.outNeighbors = outletVertex.index;
                outletVertex.inNeighbors = inletVertex.index;

                thisInNeighbor.inletVertices(end+1) = inletVertex.index;
                thisNode.outletVertices(end+1) = outletVertex.index;

                %motion-edge (corresponding to in-funnel *to* new node)
                edgeCost = C.computeCostFn(thisInNeighbor.pose,thisNode.pose);

                tempMotionEdge = edgeStruct(nan, [inletVertex.index outletVertex.index], edgeCost);
                tempMotionEdge.type = 1; %motion-edge

                obj.addEdge(tempMotionEdge);

                inletVertex.outEdges = tempMotionEdge.index;
                outletVertex.inEdges = tempMotionEdge.index;
            end

            for i=1:length(thisNode.outEdges) %inlet - newNode, outlet - neighborNode
                thisOutEdge = C.graphEdges(thisNode.outEdges(i));
                thisOutNeighbor = C.graphNodes(thisNode.outNeighbors(i)); %or C.graphNodes(C.graphEdges(thisOutEdge).child);

                %vertices
                inletVertex = augmentedVertexStruct(nan,thisNode.index,thisOutEdge.index);
                inletVertex.type = 'inlet'; inletVertex.pose = thisNode.pose;
                outletVertex = augmentedVertexStruct(nan,thisOutNeighbor.index,thisOutEdge.index);
                outletVertex.type = 'outlet'; outletVertex.pose = thisOutNeighbor.pose;

                obj.addVertex(inletVertex); %NaN index -- will get auto-assigned when added to the graph
                obj.addVertex(outletVertex);

                inletVertex.outNeighbors = outletVertex.index;
                outletVertex.inNeighbors = inletVertex.index;

                thisNode.inletVertices(end+1) = inletVertex.index;
                thisOutNeighbor.outletVertices(end+1) = outletVertex.index;

                %motion-edge (corresponding to out-funnel *from* new node)
                edgeCost = C.computeCostFn(thisNode.pose,thisOutNeighbor.pose);

                tempMotionEdge = edgeStruct(nan, [inletVertex.index outletVertex.index], edgeCost);
                tempMotionEdge.type = 1; %motion-edge

                obj.addEdge(tempMotionEdge);

                inletVertex.outEdges = tempMotionEdge.index;
                outletVertex.inEdges = tempMotionEdge.index;
            end

            addContinuityEdges(obj,F,thisNode);

            for i=1:length(thisNode.outNeighbors) %assuming outNeigbors is same as inNeighbors
                thisNeighbor = C.graphNodes(thisNode.outNeighbors(i));
                addContinuityEdges(obj,F,thisNeighbor);
            end

        end
        
        %function to check for compossibility and add continuity edges
        function obj = addContinuityEdges(obj,F,thisNode)

            for i=1:length(thisNode.outletVertices)
                thisOutletVertex = obj.graphVertices(thisNode.outletVertices(i));

                %outlet of inFunnel to node
                thisInFunnel = F.funnelEdges(thisOutletVertex.vertexData(2)); %accessing the funnel

                for j=1:length(thisNode.inletVertices)
                    tempInletVertex = obj.graphVertices(thisNode.inletVertices(j));

                    %if already checked previosuly continue 
                    %implemented by checking whether it already it's in the previous list
                    if ismember(tempInletVertex.index,thisOutletVertex.outNeighbors)
                       continue
                    end

                    %first-check to rule out the case of encountering goal region
                    %vertexData - [goalIndex (= 1) | NaN]
                    if isnan(tempInletVertex.vertexData(2)) %it's an outlet at goal, so by default
                                                             %include a continuity edge

                         tempContinuityEdge = edgeStruct(nan, [thisOutletVertex.index obj.goalVertex.index], 0); %edge-cost is zero
                         tempContinuityEdge.type = 0; %continuity-edge
                         obj.addEdge(tempContinuityEdge);

                         obj.goalVertex.inNeighbors(end+1) = thisOutletVertex.index;
                         thisOutletVertex.outNeighbors(end+1) = obj.goalVertex.index;

                         obj.goalVertex.inEdges(end+1) = tempContinuityEdge.index;
                         thisOutletVertex.outEdges(end+1) = tempContinuityEdge.index;

                         continue 
                    end

                    %inlet of outFunnel from node
                    tempOutFunnel = F.funnelEdges(tempInletVertex.vertexData(2));

                    %if F.compossible(thisInFunnel,tempOutFunnel)
                    if F.isCompossible(thisInFunnel,tempOutFunnel)    
                        tempContinuityEdge = edgeStruct(nan, [thisOutletVertex.index tempInletVertex.index], 0); %edge-cost is zero
                        tempContinuityEdge.type = 0; %continuity-edge
                        obj.addEdge(tempContinuityEdge);

                        tempInletVertex.inNeighbors(end+1) = thisOutletVertex.index;
                        thisOutletVertex.outNeighbors(end+1) = tempInletVertex.index;

                        tempInletVertex.inEdges(end+1) = tempContinuityEdge.index;
                        thisOutletVertex.outEdges(end+1) = tempContinuityEdge.index;
                    end
                end
            end
        end
        
        %-------------------------------------------------------------------------%
        %Incremental graph-search related routines
        
        %initialises the graph-search algorithm
        %all nodes have infinite g and lmc value by default (constructor definition)
        function obj = initialiseGraphSearch(obj,Q)

            obj.km = 0;
            obj.goalVertex.lmc = 0;
            obj.goalVertex.key = computeKey(obj,obj.goalVertex);
            Q.push(obj.goalVertex,obj.goalVertex.key);
            obj.previousPose = obj.startVertex;
        end
        
        %compute the heuristic from current vertex to .. 
        %goal (forward search)/start (reverse search)
        function h = computeHeuristic(obj,vertex,start)
            h = 1.0*obj.computeEuclidianDist(vertex.pose,start.pose); 
        end
        
        function obj = updatekm(obj,C)
            obj.km = obj.km + obj.computeHeuristic(C.previousRobotNode,C.currentRobotNode);
        end
        %updates the key values of any vertices and pushes it into the queue
        function obj = updateVertex(obj,Q,vertex)

            delta = 0.01; %0.5

            if vertex.index ~= obj.goalVertex.index
                vertex.lmc = computeLMC(obj,vertex);
            end 

            if vertex.inHeap
                Q.remove(vertex);
            end

            %remove and push only inconsistent nodes
            %if node.cost ~= node.lmc %if inconsistent
            if abs(vertex.cost-vertex.lmc)>delta %only if substantial change
                vertex.key = computeKey(obj,vertex);
                Q.push(vertex, vertex.key);
            end
        end
        
        %Computes (and recomputes) optimal funnel-paths 
        %Makes repairs (new sample configurations) and rewires (changed obstacle-space)
        function [status,obj] = computeShortestPath(obj,Q)

            obj.startVertex.key = computeKey(obj,obj.startVertex);

            while obj.startVertex.lmc ~= obj.startVertex.cost || ... %Q.indexOfLast > 0
                  compareStartKey(obj,Q.topKey(),obj.startVertex.key)      

                kOld = Q.topKey();
                currentVertex = Q.pop();

                %plot(currentVertex.pose(1), currentVertex.pose(2), 'oy', 'MarkerSize', 8, 'LineWidth', 3.5)
                %drawnow

                if(isempty(currentVertex)) %if the priority queue is empty, return
                    break
                end

                currentVertex.key = computeKey(obj,currentVertex);

                if compareKey(obj,kOld,currentVertex.key)
                    Q.push(currentVertex,currentVertex.key);
                elseif currentVertex.cost > currentVertex.lmc %overconsistent, shorter path exists
                    currentVertex.cost = currentVertex.lmc;
                    updateInNeighborVertices(obj,Q,currentVertex);
                else
                    currentVertex.cost = inf;
                    currentVertex.parent = nan;
                    updateInNeighborVertices(obj,Q,currentVertex);
                    updateVertex(obj,Q,currentVertex);
                end
            end
            status = ~isinf(obj.startVertex.cost); %if infinite cost, no path exists CURRENTLY

        end
        
        %adding vertex and edges to the graph data structure
        function obj = addVertex(obj,vertex)
                obj.numVertices = obj.numVertices + 1;
                obj.graphVertices(obj.numVertices) = vertex;
                if isnan(vertex.index)
                    vertex.index = obj.numVertices;
                end
        end
        
        function obj = addEdge(obj,edge)
                obj.numEdges = obj.numEdges + 1;
                obj.graphEdges(obj.numEdges) = edge;
                if isnan(edge.index)
                    edge.index = obj.numEdges;
                end
        end
          
        %plotting functions
        function drawAugmentedGraph(obj,C)
            
            nodes = NaN(obj.numVertices,2);
            for i=1:obj.numVertices
                nodes(i,:) = obj.graphVertices(i).pose;
                
                %tempVertex  = obj.graphVertices(i);
                %nodes(i,:) = C.graphNodes(tempVertex.vertexData(1)).pose;
                %1st element vertex data is node (configuration/pose)
                %2nd element is funnel
            end
            
            % a bit of data processing for faster plotting
            edges = NaN(3*obj.numEdges,2);
            for i=3:3:3*obj.numEdges
                tempEdge = obj.graphEdges(i/3);
                
                %just plot the continuity edges
                %for motion-edges (use C.drawSearchGraph())
                
                if tempEdge.type == 1 %if *not* continuity-edge
                    continue
                end
                    
                tempChildVertex = obj.graphVertices(tempEdge.child);
                edges(i-2,:) = tempChildVertex.pose; % edge starts
                
                tempParentVertex = obj.graphVertices(tempEdge.parent);
                edges(i-1,:) = tempParentVertex.pose; % edge ends
                
                someRand = 2*pi*rand();
                someLength = 0.75*[cos(someRand) sin(someRand)];
                %someLength = 0.5*rand([1 2]) + [0.25 0.25];
                edges(i-2,:) = edges(i-2,:) + someLength; % some extra length
                edges(i-1,:) = edges(i-1,:) - someLength; % for visualisation
            end 
            
            %plot(edges(:,1),edges(:,2),'b','LineWidth',3); % [0, 0.4470, 0.7410]
            plot(edges(:,1),edges(:,2),'b:','LineWidth',3); % [0, 0.4470, 0.7410]
            
            plot(nodes(:,1),nodes(:,2), 'o','Color','y'); %[0, 0.4470, 0.7410]
            
            C.drawSearchGraph();
            
        end
        
        function drawAugmentedSearchTree(obj)
            edges = NaN(3*obj.numVertices,2);
            for i=6:3:3*obj.numVertices
                
                tempVertex = obj.graphVertices(i/3); %starting from 2, because 1 is goal node which doesn't have a parent 
                
                if strcmp(tempVertex,'inlet') %parents of outlet's can't be visualised
                    continue
                end
                
                %if tempVertex.inHeap || tempVertex.lmc > obj.startVertex.cost
                %    continue
                %end
                
                if isnan(tempVertex.parent) 
                    continue
                end
                
                tempParentVertex = obj.graphVertices(tempVertex.parent); %an inlet
                
                if isnan(tempParentVertex.parent)
                    continue
                end

                edges(i-1,:) = tempParentVertex.pose; % edge ends
                edges(i-2,:) = obj.graphVertices(tempParentVertex.parent).pose; %next outlet
            end 

            %plot(edges(:,1),edges(:,2),'Color','y','LineWidth',2.7); %[0, 0.4470, 0.7410], [0.3010, 0.7450, 0.9330] 
            plot(edges(:,1),edges(:,2),'Color',[0, 0.4470, 0.7410],'LineWidth',2.5); %1.9 %[0.3010, 0.7450, 0.9330] 
              
            plot(obj.goalVertex.pose(1), obj.goalVertex.pose(2), 'xr', 'MarkerSize', 8, 'LineWidth', 3.5)
            plot(obj.startVertex.pose(1), obj.startVertex.pose(2), 'sg', 'MarkerSize', 8, 'LineWidth', 3.5)  
        end
        
        function drawPathToGoal(obj)
    
            %drawSearchTree(obj);
            start = obj.startVertex.pose;
            goal = obj.goalVertex.pose;
            delta = 0.1;

            j = 1;
            %Constructing the goal branch by backtracking through parent pointers
            temp = obj.startVertex;
            while 1

                if (isnan(temp.parent))
                %if (isempty(temp) || isnan(temp.parent))
                    disp('There does not exist a feasible path currently!');
                    return
                    %break
                end

                goalBranch(j,:) = temp.pose;  
                temp = obj.graphVertices(temp.parent);
                if abs(temp.pose - goal) < delta
                    goalBranch(j+1,:)=goal;
                    break
                end
                j = j+1;
            end

            plot(start(1),start(2), 'sg','MarkerSize', 6,'LineWidth',2);
            plot(goalBranch(:,1),goalBranch(:,2),'-.g','LineWidth', 2.5); %2.5
            plot(goal(1),goal(2),'xr','MarkerSize', 6,'LineWidth',2);
        end
    end
    
    methods (Access = private)
        %Computing Euclidian distance
        function d = computeEuclidianDist(obj,v,u)
            d = sqrt((v(1)-u(1))^2 + (v(2)-u(2))^2);
        end

        %compute the key value of the vertex
        function key = computeKey(obj,vertex)
            k1 = min(vertex.cost, vertex.lmc) + obj.computeHeuristic(vertex,obj.startVertex) + obj.km;
            k2 = min(vertex.cost, vertex.lmc);
            key = [k1 k2];
        end 
        
        % %returns TRUE if key1 is smaller than key2
        % %note that it follows a lexicographic-type comparison
        %The compareKey function has been defined in the heap class
        function status = compareKey(obj,key1, key2)
            status = false;
            delta = 0; %should be ideally zero (tune it later)
            if key1(1) < key2(1) + delta
                status = true;
            elseif key1(1) == key2(1)
                if key1(2) < key2(2) + delta
                    status = true;
                end
            end
        end

        % %returns TRUE if key1 is smaller than key2
        % Checking a bit further downstream of robot location
        function status = compareStartKey(obj,key1, key2)
            status = false;
            delta = 6; %should be approx same as the max edge cost (6)
            if key1(1) < key2(1) + delta
                status = true;
            elseif key1(1) == key2(1)
                if key1(2) < key2(2) + delta
                    status = true;
                end
            end
        end

        %finds the best lmc among the neighbors of the given node
        %lmc by default means the minimum one-step lookahead cost
        %lmc - locally minimum cost to come
        function [minLMC,obj] = computeLMC(obj,vertex)

            minLMC = inf;

            for i=1:length(vertex.outNeighbors)
                tempNeighbor = obj.graphVertices(vertex.outNeighbors(i));

                if (tempNeighbor.parent == vertex.index) %explicitly avoiding loops
                    continue
                end

                tempEdge = obj.graphEdges(vertex.outEdges(i));
                neighborCostToGoal = min(tempNeighbor.cost,tempNeighbor.lmc);

                if(neighborCostToGoal + tempEdge.cost < minLMC)
                    minLMC = tempNeighbor.cost + tempEdge.cost;
                    vertex.parent = tempNeighbor.index;
                    
                    vertex.parentFunnelEdge = vertex.vertexData(2); %updating the parent funnelEdge
                end
            end
        end

        %updates all the child vertices of a given node
        function obj = updateInNeighborVertices(obj,Q,vertex)

            for i = 1:length(vertex.inNeighbors)
                tempInNeighbor = obj.graphVertices(vertex.inNeighbors(i));
                updateVertex(obj,Q,tempInNeighbor);
            end
        end

        %updates all the child vertices of a given node
        function obj = updateOutNeighborVertices(obj,Q,vertex)

            for i = 1:length(vertex.outNeighbors)
                tempOutNeighbor = obj.graphVertices(vertex.outNeighbors(i));
                updateVertex(obj,Q,tempOutNeighbor);
            end
        end
    end
end