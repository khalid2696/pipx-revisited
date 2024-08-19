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

%Defining a class to abstract information about funnels in the funnel-network  
classdef searchFunnel < handle
    properties
        
        funnelLibrary
        stateDimension
        extendDistance
        resolution
        configXArray
        configYArray
        
        numNodes 
        numFunnelEdges
        
        graphNodes
        funnelEdges
        
        startNode
        goalNode
    end
    methods
        function obj = searchFunnel(library,libraryResolution)
            
            obj.startNode = []; %will be updated in runtime
            obj.goalNode  = []; %will be updated in runtime
                     
            obj.numNodes = 0;
            obj.numFunnelEdges = 0;

            obj.graphNodes = nodeStruct();
            obj.funnelEdges = funnelStruct();
            
            obj.funnelLibrary = library;
            obj.stateDimension = 6; %no. of states
            obj.extendDistance = 5;

            if strcmp(libraryResolution,'dense') %1-Sparse %0.5-Nominal %0.25-Dense
                obj.resolution = 0.25;
            elseif strcmp(libraryResolution,'sparse')
                obj.resolution = 1;
            else
                obj.resolution = 0.5;
            end
        
            obj.configXArray = -obj.extendDistance:obj.resolution:obj.extendDistance;
            obj.configYArray = -obj.extendDistance:obj.resolution:obj.extendDistance; 
        end
        
        function obj = addNode(obj,node)
                obj.numNodes = obj.numNodes + 1;
                obj.graphNodes(obj.numNodes) = node;
        end
        
        function obj = addFunnelEdge(obj,funnel)
                obj.numFunnelEdges = obj.numFunnelEdges + 1;
                obj.funnelEdges(obj.numFunnelEdges) = funnel;
                if isnan(funnel.index)
                    funnel.index = obj.numFunnelEdges;
                end
        end
        
        function trajectoryLength = computeNominalTrajectoryLength(obj,funnel)
            trajectoryLength = 0;

            for i=2:length(funnel.trajectory)
                ds = norm(funnel.trajectory(1:2,i) - funnel.trajectory(1:2,i-1));
                trajectoryLength = trajectoryLength + ds;
            end
        end

        %constructing the funnel network
        function flag = constructFunnelNetwork(obj,T,C,O,newNode,neighbors)

            N = length(neighbors);
            flag = 0;
            maxNeighborsAllowed = 8;
            delta = 0.5;

            if (N < 1) %if no neighbor return
                flag = 1;
                return
            end
            
            %restricting the max number of neighbors
            if N > maxNeighborsAllowed
                N = maxNeighborsAllowed;
                %disp('\n Neighbors pruned!')
            end

            prevEdgeCount = obj.numFunnelEdges;
            
            for i=1:N
                thisNeighbor = neighbors{i};

                if thisNeighbor.withinObstacle
                    continue
                end
                
                %if it's "too small" of a distance (delta), continue
                if obj.euclidianDist(thisNeighbor.pose,newNode.pose) < delta
                    %disp('A "small-hop" neighbor encountered.. discarding it!')
                    continue
                end
        
                %outFunnel for newNode/ inFunnel for neighborNode 
                newFunnel = obj.steer(newNode,thisNeighbor.pose);

                if(~O.funnelCollisionFree(newFunnel) || ~O.edgeCollisionFree(thisNeighbor.pose,newNode.pose))
                    continue
                end           
            
                %Add the edge to the search tree
                funnelEdge = funnelStruct(nan,newFunnel);

                %funnels and edges direction are swapped (because of reverse search)
                funnelEdge.child = thisNeighbor.index; funnelEdge.parent = newNode.index;

                edgeCost = C.computeCostFn(thisNeighbor.pose,newNode.pose);
                funnelEdge.cost = edgeCost;
                funnelEdge.nominalCost = obj.computeNominalTrajectoryLength(funnelEdge);

                obj.addFunnelEdge(funnelEdge);

                %adding in and out Neighbors
                thisNeighbor.outNeighbors(end+1) = newNode.index;
                newNode.inNeighbors(end+1) = thisNeighbor.index;     


                %adding out and in edges
                tempEdge = edgeStruct(nan, [thisNeighbor.index newNode.index], edgeCost);
                tempEdge.type = 1; %motion-edge

                C.addEdge(tempEdge);
                thisNeighbor.outEdges(end+1) = tempEdge.index;
                newNode.inEdges(end+1) = tempEdge.index;

                %outFunnel for neighbor/ inFunnel for newNode 
                newFunnel = obj.steer(thisNeighbor,newNode.pose);

                if(~O.funnelCollisionFree(newFunnel) || ~O.edgeCollisionFree(thisNeighbor.pose,newNode.pose))
                    continue
                end

                %Add the edge to the search tree
                funnelEdge = funnelStruct(nan,newFunnel);

                %funnels and edges direction are swapped (because of reverse search)
                funnelEdge.child = newNode.index; funnelEdge.parent = thisNeighbor.index;

                edgeCost = C.computeCostFn(newNode.pose,thisNeighbor.pose);
                funnelEdge.cost = edgeCost;
                funnelEdge.nominalCost = obj.computeNominalTrajectoryLength(funnelEdge);

                obj.addFunnelEdge(funnelEdge);

                thisNeighbor.inNeighbors(end+1) = newNode.index;
                newNode.outNeighbors(end+1) = thisNeighbor.index; 

                tempEdge = edgeStruct(nan, [newNode.index thisNeighbor.index], edgeCost);
                tempEdge.type = 1; %motion-edge

                C.addEdge(tempEdge);
                thisNeighbor.inEdges(end+1) = tempEdge.index;
                newNode.outEdges(end+1) = tempEdge.index;
            end

            if prevEdgeCount == obj.numFunnelEdges %i.e. if no new edge was added
                flag = 1;
                return
            end

            %adding to the data structures
            C.addNode(newNode); obj.addNode(newNode);
            T.kdInsertAsPayload(newNode);
        end
        
        %new functions added (Jul '24)
        function shiftedFunnel = steer(obj,parentNode,desiredConfig)
    
            funnel = obj.findFunnel(parentNode.pose,desiredConfig); 
        
            shiftVector = [parentNode.pose 0]; %start point -- x, y and z
            shiftedFunnel = obj.shiftAlongCyclicCoordinates(funnel,shiftVector);
        
        end

        %Extracting the funnel-edge (parent to sampled node) from the trajectory library
        function funnel = findFunnel(obj,parentConfig,desiredConfig) 
        
            deltaQ = desiredConfig - parentConfig; %config space (q) --> [x,y]
            
            [~, closestXIndex] = min(abs(obj.configXArray - deltaQ(1)));  
            [~, closestYIndex] = min(abs(obj.configYArray - deltaQ(2)));
        
            dictionaryKey = [obj.configXArray(closestXIndex), obj.configYArray(closestYIndex)];
        
            funnel = obj.funnelLibrary(num2str(dictionaryKey));
        end 

        %new functions added (Jul '24)
        function shiftedFunnel = shiftAlongCyclicCoordinates(obj,funnel,cyclicCoords)
    
            N = length(funnel.time);
            
            %instantiating an empty struct
            shiftedFunnel = struct('time',funnel.time,'trajectory',NaN(N,6),'RofA',funnel.RofA);
            
            %defining the shift vector
            shiftArray = ones(N,1)*[cyclicCoords 0 0 0]; %no shift along velocity!
            shiftedFunnel.trajectory = funnel.trajectory + shiftArray';
        end

        %checks whether funnel1 is compossible with funnel2
        %that is if outlet of funnel1 is contained within the inlet of funnel2
        function check = isCompossible(obj,funnel1,funnel2)
        
            %check = 1;  
            
            %outletRofA = funnel1.RofA(:,:,end); %index 'end': outlet of funnel 1
            outletCenter = funnel1.trajectory(:,end); %index 'end': outlet of funnel 1
         
            inletRofA = funnel2.RofA(:,:,1); %index 1: inlet of funnel 2
            inletCenter = funnel2.trajectory(:,1); %index 1: inlet of funnel 2
            
            %first pass check
            if(outletCenter-inletCenter)'*inletRofA*(outletCenter-inletCenter)>1 %if the centre itself doesn't lie in the ellipse, return  
                check = 0;
                %disp('Out in the first pass itself! (centre doesnot lie inside)')
                return
            end
            
            checkPoints = obj.decomposeOutletIntoEllipses(funnel1);
            
            check = obj.ellipsoidinEllipsoidCheck(inletCenter,inletRofA,checkPoints);
        end
        
        %-------------------------------------------------------------------------%
        %Ellipsoid decomposition functions   
        function checkPoints = decomposeOutletIntoEllipses(obj,funnel)
            
            %numDimensions = 6; %6 dimensional state space
            
            checkResolution = 13; %keep it as an odd number preferably
            th = linspace(-pi,pi,checkResolution);
            
            numProjections = obj.stateDimension*(obj.stateDimension-1)/2;
            checkPoints = zeros(obj.stateDimension,checkResolution,numProjections); %2 because 2D ellipses
            
            %accessing the funnel's outlet properties
            %outletRofA = reshape(funnel.RofA(end,:,:),numDimensions,numDimensions);
            outletRofA = funnel.RofA(:,:,end);
            outletCenter = funnel.trajectory(:,end); %index 'end': outlet of funnel 1
            
            count = 1;
            
            for i = 1:obj.stateDimension-1
                for j = i+1:obj.stateDimension
                    
                    tempEllipsoid = inv(outletRofA);
                    
                    %accessing the corresponding elements
                    E(1,1) = tempEllipsoid(i,i); E(1,2) = tempEllipsoid(i,j);
                    E(2,1) = tempEllipsoid(j,i); E(2,2) = tempEllipsoid(j,j);

                    %Getting the checkpoints on the boundary of the ellipse
                    ell = E^(1/2)*[cos(th); sin(th)];
                    %ell = sqrtm(E)*[cos(th); sin(th)];
                    
                    checkPoints(:,:,count) = outletCenter * ones(1,checkResolution);
                    checkPoints(i,:,count) = ell(1,:) + checkPoints(i,:,count); %shifting origin
                    checkPoints(j,:,count) = ell(2,:) + checkPoints(j,:,count); %shifting origin
        
                    count = count+1; %keeping track of number of projections
                end
            end
        end

        function check = ellipsoidinEllipsoidCheck(obj,inletCenter,inletRofA,checkPoints)

            check = 1;
            
            numDimensions = size(checkPoints,1);
            checkResolution = size(checkPoints,2);
            numProjections = size(checkPoints,3);
            
            %reshaping the matrix for ease of use
            checkPoints = reshape(checkPoints,numDimensions,checkResolution*numProjections);
            
            for i=1:size(checkPoints,2)
                thisCheckPoint = checkPoints(:,i);
                
                if(thisCheckPoint-inletCenter)'*inletRofA*(thisCheckPoint-inletCenter) > 1.1 %some extra allowance to account for numerical errors
                    check=0;
                    %(thisCheckPoint-inletCenter)'*inletRofA*(thisCheckPoint-inletCenter)
                    return
                end
            end
            
        end

        function check = inFunnel(obj,point)

            check = 0;

            for i=1:obj.numFunnelEdges
                thisFunnel = obj.funnelEdges(i);
                traj = thisFunnel.trajectory;
                funnel = thisFunnel.RofA;        
                
                %at this stage you have the trajectory and
                %the RofA along the knot points
                if(notInBoudingCircle(obj,traj,funnel,point))
                    continue
                end 

                funnelSize = length(thisFunnel.time);
                vanDerSequence = ceil(vdcorput(obj,funnelSize,2)*funnelSize);
                
                for j=1:funnelSize
                    state = traj(1:2,vanDerSequence(j));
                    RofA = funnel(:,:,vanDerSequence(j));
                    if(inBasin(obj,state,RofA,point))
                        check = 1;        
                        return
                    end
                end
            end
        end
        
        %new function added!
        function check = inAnyInlets(obj,configurationPose) %2D configuration for now

            check = 0;
            
            for i=1:obj.numFunnelEdges
                
                thisFunnel = obj.funnelEdges(i);
                
                if obj.notInBoudingCircle(thisFunnel.trajectory,thisFunnel.RofA,configurationPose)
                    continue
                end 
            
                if inFunnelInlet(obj,thisFunnel,configurationPose)
                    check = 1;        
                    return
                end
            end
        end
        
        %new function added!
        function check = inFunnelInlet(obj,funnel,configurationPose) %2D configuration for now

            check = 0;
            
            inletCenter = funnel.trajectory(1:2,1); %1 is inlet
            inletRegion = funnel.RofA(:,:,1); %1 is inlet
            
            if(inBasin(obj,inletCenter,inletRegion,configurationPose))
                check = 1;        
                return
            end
        end
        
        %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%
            %New Additions here
        %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%
                
        function examineSolutionFunnelBranchWithPlots(obj)

            if isnan(obj.startNode.parentFunnelEdge)
                return %Reached the goal region, return
            end

            funnel1 = obj.funnelEdges(obj.startNode.parentFunnelEdge);
            thisNode = obj.graphNodes(obj.startNode.parent);
            obj.drawFunnel(funnel1,2)

            while 1        
                if thisNode.index == obj.goalNode.index
                    break
                end

                tempFunnelIndex = thisNode.parentFunnelEdge;
                funnel2 = obj.funnelEdges(tempFunnelIndex);

                obj.drawFunnel(funnel2,2)

                obj.drawEllipse(funnel1.trajectory(:,end),funnel1.RofA(:,:,end),0); %end is the outlet
                obj.drawEllipse(funnel2.trajectory(:,1),funnel2.RofA(:,:,1),2); %1 is the inlet

                check = obj.isCompossible(funnel1,funnel2);
                
                if ~check
                    disp('Compossibility check failed in the solution funnel-path!!')
                end

                thisNode = obj.graphNodes(thisNode.parent);
                funnel1 = funnel2;
            end
        end
        
        function constructShortestFunnelPath(obj,G)
            
            thisVertex = G.startVertex;
            
            if isinf(thisVertex.cost) %if shortest path doesn't exist return
                return
            end
            
            while 1        
                thisVertex.index;
                if thisVertex.index == G.goalVertex.index
                    break
                end
                
                if strcmp(thisVertex.type,'outlet') %sufficient to look at only funnels
                    thisVertex = G.graphVertices(thisVertex.parent);
                    continue
                end
                
                tempNode = obj.graphNodes(thisVertex.vertexData(1));
                tempFunnel = obj.funnelEdges(thisVertex.vertexData(2));
                tempNode.parentFunnelEdge = tempFunnel.index;
                
                thisVertex = G.graphVertices(thisVertex.parent);
                
                tempNode.parent = thisVertex.vertexData(1);
            end
        end
        
        %-------------------------------------------------------%
        %plotting functions
        %-------------------------------------------------------%
        function drawSearchFunnel(obj)
            
            for i=1:obj.numNodes
                thisNode = obj.graphNodes(i);
                
                if thisNode.withinObstacle || isnan(thisNode.bestInlet)
                    continue
                end
                
                thisFunnel = obj.funnelEdges(thisNode.parentFunnelEdge);
                
                if ~thisFunnel.withinObstacle
                    drawFunnel(obj,thisFunnel,1);
                end
            end

            plot(obj.goalNode.pose(1), obj.goalNode.pose(2), 'xr', 'MarkerSize', 8, 'LineWidth', 3.5)
            plot(obj.startNode.pose(1), obj.startNode.pose(2), 'sg', 'MarkerSize', 8, 'LineWidth', 3.5)
            %plot(C.startNode.pose(1), C.startNode.pose(2), 'dm', 'MarkerSize', 6, 'LineWidth', 3.5)
        end
        
        function drawSearchTrajectories(obj)
            
            for i=1:obj.numNodes
                thisNode = obj.graphNodes(i);
                
                if thisNode.withinObstacle || isnan(thisNode.bestInlet)
                    continue
                end
                
                thisFunnel = obj.funnelEdges(thisNode.parentFunnelEdge);
                
                if ~thisFunnel.withinObstacle
                    traj = thisFunnel.trajectory';
                    plot(traj(:,1),traj(:,2),'-.b','LineWidth',2.5);
                    
                    %funnel-inlet
                    obj.drawEllipse(thisFunnel.trajectory(:,1),thisFunnel.RofA(:,:,1),1);
                    
                    %funnel-outlet
                    obj.drawEllipse(thisFunnel.trajectory(:,end),thisFunnel.RofA(:,:,end),2);
                    
                    %Plotting the end and start points
                    plot(traj(1,1),traj(1,2),'oy','LineWidth',2.5,'MarkerSize',5); %inlet to the funnel
                    plot(traj(end,1),traj(end,2),'xg','LineWidth',3,'MarkerSize',7); %outlet of the funnel
                end
            end
        end
                
        %draws the goal branch by backtracking through the parent pointers
        function drawGoalBranch(obj)

            start = obj.startNode.pose;
            goal = obj.goalNode.pose;
            delta = 0.01;
            
            %Constructing the goal branch by backtracking through parent pointers
            tempNode = obj.startNode;
            while 1
                
                if abs(tempNode.pose - goal) < delta
                    break
                end
                
                if (isnan(tempNode.parent))
                    disp('There does not exist a feasible path currently!');
                    return
                    %break
                end

                tempFunnel = obj.funnelEdges(tempNode.parentFunnelEdge);
                
                %P = tempFunnel.RofA;
                x = tempFunnel.trajectory;
                drawFunnel(obj,tempFunnel,2);
                plot(x(1,:),x(2,:),'-.c','LineWidth',2.5);
                
                tempNode = obj.graphNodes(tempNode.parent);
            end

            %plot(start(1),start(2),'sg','LineWidth',3,'MarkerSize',7);
            plot(start(1),start(2),'dm', 'MarkerSize', 6, 'LineWidth', 3.5);
            plot(goal(1),goal(2),'xr','LineWidth', 3,'MarkerSize',7);
           
        end
        
        %draws funnel defined by trajectory, x and ellipsoids, P along the knot points
        function drawFunnel(obj,funnel,status)
            N = length(funnel.trajectory);
            for j=N-10:-3:1 %change it to -1 to get more pretty plots
                P = funnel.RofA(:,:,j);
                xt = funnel.trajectory(1:2,j);
                drawEllipse(obj,xt,P,status);
            end

            if status==0
                drawDeletedTrajectory(obj,funnel);
            else
                drawTrajectory(obj,funnel);
            end
        end

        %draws trajectory, x in 2D space
        function drawTrajectory(obj,funnel)
            traj = funnel.trajectory';
            plot(traj(:,1),traj(:,2),':k','LineWidth',1.4);
            %Plotting the end and start points
            plot(traj(1,1),traj(1,2),'ok','LineWidth',1.5,'MarkerSize',3); %inlet to the funnel
            plot(traj(end,1),traj(end,2),'.k','LineWidth',2,'MarkerSize',4); %outlet of the funnel
        end

        %draws deleted trajectory, x in white
        function drawDeletedTrajectory(obj,funnel)
            traj = funnel.trajectory';
            plot(traj(:,1),traj(:,2),':w','LineWidth',2.4);
            %Plotting the end and start points
            plot(traj(1,1),traj(1,2),'ow','LineWidth',1.5,'MarkerSize',3); %inlet to the funnel
            %plot(trajectory(end,1),trajectory(end,2),'.k','LineWidth',2,'MarkerSize',4); %interior of the funnel
        end

        %draws an ellipse defined by xTMx<1 with centre c
        %status - 2 - goal branch; 1 - normal edge; 0 - deleted edge
        function drawEllipse(obj,center,RofA,status)

            N = 12; %50 for more pretty plots
            th = linspace(-pi,pi,N);
            Basis = [1 0; 0 1; 0 0; 0 0; 0 0; 0 0]; %xy
            %Basis = [0 0; 0 1; 0 1; 0 0; 0 0; 0 0]; %yz
            %Basis = [1 0; 0 0; 0 1; 0 0; 0 0; 0 0];  %zx
            E = Basis'/RofA*Basis;
            %ell = E^(1/2)*[cos(th); sin(th)];
            ell = sqrtm(E)*[cos(th); sin(th)];

            if status == 2 
                color = [0 0.9 0.1]; alpha = 0.5; %green
            elseif status == 1
                color = [0.8 0.8 0.8]; alpha = 0.5; %gray
            else
                color = [0.99 0.99 0.99]; alpha = 0.8; %opaque
            end

            fill(center(1) + ell(1,:),center(2) + ell(2,:),color,'edgeColor',color,'FaceAlpha',alpha);
        end

        %3D - (x,y) + time - counterparts of the former functions
        %draws the funnel with time-trajectory and ellipsoids information at the knot points
        function drawFunnelwithTime(obj,funnel)
            %figure(2)
            N = length(funnel.trajectory);
            for j=N:-1:1
                P = funnel.RofA(:,:,j);
                xt = funnel.trajectory(:,j);
                t = funnel.time(:,j);
                drawEllipsewithTime(obj,t,xt,P);
            end
            drawTrajectorywithTime(obj,funnel); 
        end

        %draws trajectory with time in a 3D plot
        function drawTrajectorywithTime(obj,funnel)
            %figure(2)
            traj = funnel.trajectory';
            plot3(traj(:,1),traj(:,2),obj.time,'--k','LineWidth',1.5);
            %Plotting the end and start points
            plot3(traj(1,1),traj(1,2),obj.time(1),'ok','LineWidth',1.5,'MarkerSize',3);
            plot3(traj(end,1),traj(end,2),obj.time(end),'.k','LineWidth',2,'MarkerSize',4);
        end

        %draws 2D ellipses "elevated" along the time axis
        %status - 2 - goal branch; 1 - normal edge; 0 - deleted edge
        function drawEllipsewithTime(obj,time,center,RofA,status)
            %figure(2);
            hold on
            N = 50;
            th = linspace(-pi,pi,N);
            Basis = [1 0; 0 1; 0 0; 0 0; 0 0; 0 0]; %xy
            %Basis = [0 0; 0 1; 0 1; 0 0; 0 0; 0 0]; %yz
            %Basis = [1 0; 0 0; 0 1; 0 0; 0 0; 0 0];  %zx
            E = Basis'/RofA*Basis;
            ell = E^(1/2)*[cos(th); sin(th)];
            time = time*ones(length(ell),1);

            if status == 2
                color = [0 0.9 0.1]; 
            elseif status == 1
                color = [0.8 0.8 0.8];
            else
                color = [1 1 1];
            end

            fill3(center(1) + ell(1,:),center(2) + ell(2,:),time, color,'edgeColor',color,'FaceAlpha',0.1);
        end   

        %draws the goal branch along the time axis as well
        function drawGoalBranchwithTime(obj,searchGraph)
            %figure(2)
  
            %Constructing the goal branch by backtracking through parent pointers
            start = searchGraph.startNode.pose;
            goal = searchGraph.goalNode.pose;
            delta = 0.001;
            
            %Constructing the goal branch by backtracking through parent pointers
            temp = searchGraph.startNode;
            finishTime = temp.timeToGoal;
            
            while 1
                if abs(temp.pose - goal) < delta
                    break
                end
                
                if (isnan(temp.parent))
                    disp('There does not exist a feasible path currently!');
                    return
                    %break
                end
                
                tempFunnel = obj.funnelEdges(temp.bestInlet);
                x = tempFunnel.trajectory;
                t = tempFunnel.time;
                
                drawFunnelWithTime(obj,tempFunnel,2);
                plot3(x(:,1),x(:,2),t,'-.m','LineWidth',3);
                  
                temp = searchGraph.graphNodes(temp.parent);
                
            end

            plot3(start(1),start(2),finishTime,'sg','LineWidth',3,'MarkerSize',7);
            plot3(goal(1),goal(2),0,'xr','LineWidth', 3,'MarkerSize',7);
        end
        
    end
    
    %--------------------------------------------------------
    %Private access function definitions
    %--------------------------------------------------------
        
    methods (Access = private)
        
        %distance function
        function dist = euclidianDist(obj,v,w)
            dist = sqrt((v(1)-w(1))^2 + (v(2)-w(2))^2);
            %dist = sqrt(sum((v - w).^2,2));
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
        function pass = notInBoudingCircle(obj,x,RofA,point)
            pass = 0;
            initialState  = x(1:2,1);
            finalState = x(1:2,end);
            midState = (initialState+finalState)/2; %computing the approx centre of the trajectory

            radius = 1.2*euclidianDist(obj,initialState,finalState)/2;
            if(euclidianDist(obj,midState,point)>radius)
                pass = 1;
            end
        end
        
        %Function to determine whether a point lies inside an ellipse or not
        function check = inBasin(obj,x,RofA,point)
            check = 0;
            Basis = [1 0; 0 1; 0 0; 0 0; 0 0; 0 0]; %xy
            %Basis = [0 0; 0 1; 0 1; 0 0; 0 0; 0 0]; %yz
            %Basis = [1 0; 0 0; 0 1; 0 0; 0 0; 0 0];  %zx

            E = Basis'*(RofA\Basis);   %Basis'*inv(RofA)*Basis
            %P = inv(E);
            if(point'-x)'*(E\(point'-x)) < 1 % (point'-x)'*inv(E)*(point'-x)
                check=1;
                return
            end
        end
    end
end