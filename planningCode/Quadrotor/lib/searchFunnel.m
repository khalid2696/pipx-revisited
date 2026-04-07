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
        extendDistance
        resolution
        configXArray
        configYArray
        
        %dimensions of the various spaces: state, configuration & workspace
        stateSpaceDimensionIndices
        CspaceDimensionIndices
        workspaceDimensionIndices

        numNodes 
        numFunnelEdges
        
        graphNodes
        funnelEdges
        
        startNode
        goalNode
    end
    methods
        function obj = searchFunnel(library, extendDistance, libraryResolution)
            
            obj.startNode = []; %will be updated in runtime
            obj.goalNode  = []; %will be updated in runtime
                     
            obj.numNodes = 0;
            obj.numFunnelEdges = 0;

            obj.graphNodes = nodeStruct();
            obj.funnelEdges = funnelStruct();
            
            obj.funnelLibrary = library;
            obj.extendDistance = extendDistance;
            
            obj.resolution = 1/libraryResolution;
            
            % Previous implementation
            % if strcmp(libraryResolution,'dense') %1-Sparse %0.5-Nominal %0.25-Dense
            %     obj.resolution = 0.25;
            % elseif strcmp(libraryResolution,'sparse')
            %     obj.resolution = 1;
            % else
            %     obj.resolution = 0.5;
            % end
        
            obj.configXArray = -obj.extendDistance:obj.resolution:obj.extendDistance;
            obj.configYArray = -obj.extendDistance:obj.resolution:obj.extendDistance;

            obj.stateSpaceDimensionIndices = 1:12;     %12-state system
            obj.CspaceDimensionIndices = [1 2 3];  %x-y-z configuration space
            obj.workspaceDimensionIndices = [1 2]; %x-y workspace

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
            
            configSpaceTrajectory = funnel.trajectory_configurationSpace;

            trajectoryLength = 0;
            for i=2:length(configSpaceTrajectory)
                ds = norm(configSpaceTrajectory(:,i) - configSpaceTrajectory(:,i-1));
                trajectoryLength = trajectoryLength + ds;
            end
        end

        %constructing the funnel network
        function flag = constructFunnelNetwork(obj,T,C,W,newNode,neighbors)

            N = length(neighbors);
            flag = 0;
            %maxNeighborsAllowed = 12;
            delta = 1;

            if (N < 1) %if no neighbor return
                flag = 1;
                return
            end
            
            %restricting the max number of neighbors: for faster runtime (not required)
            %if N > maxNeighborsAllowed
            %    N = maxNeighborsAllowed;
            %    %disp('\n Neighbors pruned!')
            %end

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
                
                %---------------------------------------------------------------%
                % outFunnelEdges from newNode <--> inFunnelEdges to neighborNode 
                %---------------------------------------------------------------%
                %rough sanity check before subsequent computations
                %Note: make sense only for "almost-holonomic" robots
                if ~W.edgeCollisionFree(thisNeighbor.pose,newNode.pose)
                    continue
                end

                %outFunnel for newNode/ inFunnel for neighborNode 
                funnelEdge = obj.steer(newNode,thisNeighbor.pose);

                if ~W.funnelCollisionFree(funnelEdge) %|| ~W.edgeCollisionFree(thisNeighbor.pose,newNode.pose))
                    continue
                end           

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

                %----------------------------------------------------------------%
                % inFunnelEdges to newNode <--> outFunnelEdges from neighborNodes 
                %----------------------------------------------------------------%
                %rough sanity check before subsequent computations
                %Note: make sense only for "almost-holonomic" robots
                if ~W.edgeCollisionFree(thisNeighbor.pose,newNode.pose)
                    continue
                end

                funnelEdge = obj.steer(thisNeighbor,newNode.pose);

                if ~W.funnelCollisionFree(funnelEdge) %|| ~W.edgeCollisionFree(thisNeighbor.pose,newNode.pose))
                    continue
                end

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
        function funnelEdge = steer(obj,parentNode,desiredConfig)
    
            %funnel = obj.findFunnel(parentNode.pose,desiredConfig);
            %shiftVector = [parentNode.pose 0]; %start point -- x, y and z

            funnel = obj.findFunnel(desiredConfig, parentNode.pose);
            
            %instantiating an empty struct
            funnelEdge = funnelStruct(); %id would be assigned later
            funnelEdge.time = funnel.time_instances;
            
            %assigning the trajectory
            funnelEdge.trajectory_stateSpace = funnel.trajectory; %just for initialisation
            %shifting the trajectory along the cyclic coordinates
            shiftVector = [desiredConfig 0]; %cyclic coordinates -- x, y and z
            funnelEdge.trajectory_stateSpace = obj.shiftAlongCyclicCoordinates(funnelEdge,shiftVector);
            
            %assigning the invariant sets
            funnelEdge.invariantSet_stateSpace = funnel.invarianceCertificates;

            %computing projections onto configuration space and workspace for later use
            %project the funnel in state-space to C-space
            [funnelEdge.trajectory_configurationSpace, funnelEdge.invariantSet_configurationSpace] = ...
                obj.projectFunnel_nD_to_mD(funnelEdge, obj.CspaceDimensionIndices);

            %project the funnel in state-space to workspace
            [funnelEdge.trajectory_workSpace, funnelEdge.invariantSet_workSpace] = ...
                obj.projectFunnel_nD_to_mD(funnelEdge, obj.workspaceDimensionIndices);
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
        function shiftedTrajectory = shiftAlongCyclicCoordinates(obj,funnelEdge,cyclicCoords)
              
            %Constructing the shift matrix
            shiftVector = zeros(size(funnelEdge.trajectory_stateSpace,1), 1);
            
            for i=1:length(obj.CspaceDimensionIndices)
                shiftVector(obj.CspaceDimensionIndices(i)) = cyclicCoords(i);
            end

            shiftArray = ones(length(funnelEdge.time),1)*shiftVector'; %no shift along velocity!      
            
            %transposing: N * nx -> nx * N (to match convention)
            shiftArray = shiftArray';
            
            %shifted trajectory
            shiftedTrajectory = funnelEdge.trajectory_stateSpace + shiftArray;
        end

        %--------------------------------------------------------------------------------%
        % modified compossibility check - 2 options: SDP or surface-sampling (24 Oct '25)
        %--------------------------------------------------------------------------------%
        
        %checks whether funnel1 is composable with funnel2
        %that is if outlet of funnel1 is contained within the inlet of funnel2
        function check = isComposable(obj, funnel1, funnel2, checkingMethod)
        
            %check = 1;  
            if nargin < 4
                checkingMethod = 'Sampling';
            end
            
            outletRofA = funnel1.invariantSet_stateSpace(:,:,end); %index 'end': outlet of funnel 1
            outletCenter = funnel1.trajectory_stateSpace(:,end); %index 'end': outlet of funnel 1
         
            inletRofA = funnel2.invariantSet_stateSpace(:,:,1); %index 1: inlet of funnel 2
            inletCenter = funnel2.trajectory_stateSpace(:,1); %index 1: inlet of funnel 2
            
            %first pass check
            if(outletCenter-inletCenter)'*inletRofA*(outletCenter-inletCenter)>1 %if the centre itself doesn't lie in the ellipse, return  
                check = 0;
                %disp('Out in the first pass itself! (centre doesnot lie inside)')
                return
            end

            if strcmpi(checkingMethod, 'SDP')
                check = obj.isComposable_usingSDP(inletRofA, inletCenter, outletRofA, outletCenter);
            else
                numSamplePoints = 1000;
                check = obj.isComposable_usingSurfaceSampling(inletRofA, inletCenter, outletRofA, outletCenter, numSamplePoints);
            end

            %check

        end
        
        %new function added!
        function check = inAnyInlets(obj,configurationPose) %2D configuration for now

            check = 0;
            
            for i=1:obj.numFunnelEdges
                
                thisFunnel = obj.funnelEdges(i);
                
                if obj.notInBoudingCircle(thisFunnel.trajectory_workSpace, configurationPose)
                    continue
                end 
            
                if obj.inFunnelInlet(thisFunnel,configurationPose)
                    check = 1;        
                    return
                end
            end
        end
        
        %new function added!
        function check = inFunnelInlet(obj,funnel,configurationPose) %2D configuration for now

            check = 0;
            
            inletCenter_workspaceProjected = funnel.trajectory_workSpace(:,1); %1 is inlet
            inletRegion_workspaceProjected = funnel.invariantSet_workSpace(:,:,1); %1 is inlet
            
            if(obj.inBasin(inletCenter_workspaceProjected,inletRegion_workspaceProjected,configurationPose))
                check = 1;        
                return
            end
        end
        
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

                obj.drawEllipse(funnel1.trajectory_workSpace(:,end),funnel1.invariantSet_workSpace(:,:,end),0); %end is the outlet
                obj.drawEllipse(funnel2.trajectory_workSpace(:,1),funnel2.invariantSet_workSpace(:,:,1),2); %1 is the inlet

                check = obj.isComposable(funnel1,funnel2);
                
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
                    traj = thisFunnel.trajectory_workSpace;
                    plot(traj(1,:),traj(2,:),'-.b','LineWidth',2.5);
                    
                    %funnel-inlet
                    obj.drawEllipse(traj(:,1),thisFunnel.invariantSet_workSpace(:,:,1),1);
                    
                    %funnel-outlet
                    obj.drawEllipse(traj(:,end),thisFunnel.invariantSet_workSpace(:,:,end),2);
                    
                    %Plotting the end and start points
                    plot(traj(1,1),traj(2,1),'oy','LineWidth',2.5,'MarkerSize',5); %inlet to the funnel
                    plot(traj(1,end),traj(2,end),'xg','LineWidth',3,'MarkerSize',7); %outlet of the funnel
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
                
                x = tempFunnel.trajectory_workSpace;
                drawFunnel(obj,tempFunnel,2);
                plot(x(1,:),x(2,:),'-.c','LineWidth',1.5);
                
                tempNode = obj.graphNodes(tempNode.parent);
            end

            %plot(start(1),start(2),'sg','LineWidth',3,'MarkerSize',7);
            plot(start(1),start(2),'dm', 'MarkerSize', 4, 'LineWidth', 2.5);
            plot(goal(1),goal(2),'xr','LineWidth', 3,'MarkerSize',7);
           
        end
        
        %draws funnel defined by trajectory, x and ellipsoids, P along the knot points
        function drawFunnel(obj,funnel,status)
            if nargin < 3
                status = 1; %gray-colored funnels
            end

            N = length(funnel.time);
            for k=N:-1:1 %change it to -1 to get more pretty plots
                P = funnel.invariantSet_workSpace(:,:,k);
                xt = funnel.trajectory_workSpace(:,k);
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
            traj = funnel.trajectory_workSpace';
            plot(traj(:,1),traj(:,2),':k','LineWidth',1.4);
            %Plotting the end and start points
            plot(traj(1,1),traj(1,2),'ok','LineWidth',1.5,'MarkerSize',3); %inlet to the funnel
            plot(traj(end,1),traj(end,2),'.k','LineWidth',2,'MarkerSize',4); %outlet of the funnel
        end

        %draws deleted trajectory, x in white
        function drawDeletedTrajectory(obj,funnel)
            traj = funnel.trajectory_workSpace';
            plot(traj(:,1),traj(:,2),':w','LineWidth',2.4);
            %Plotting the end and start points
            plot(traj(1,1),traj(1,2),'ow','LineWidth',1.5,'MarkerSize',3); %inlet to the funnel
            %plot(trajectory(end,1),trajectory(end,2),'.k','LineWidth',2,'MarkerSize',4); %interior of the funnel
        end


        %draws an ellipse defined by xTMx<1 with centre c
        %status - 2 - goal branch; 1 - normal edge; 0 - deleted edge
        function drawEllipse(obj, ellipseCenter, ellipseMatrix, status)
            
            [eig_vec, eig_val] = eig(ellipseMatrix);
            
            theta = linspace(0, 2*pi, 20); % Parameterize ellipse
            ellipse_boundary = eig_val^(-1/2) * [cos(theta); sin(theta)];
            rotated_ellipse = eig_vec * ellipse_boundary;
            
            % plot(ellipseCenter(1) + rotated_ellipse(1, :), ...
            %      ellipseCenter(2) + rotated_ellipse(2, :), ...
            %      '-k', 'LineWidth', 1.2);  

            if status == 2 
                color = [0 0.9 0.1]; alpha = 0.8; %green
            elseif status == 1
                color = [0.8 0.8 0.8]; alpha = 0.7; %gray
            else
                color = [0.99 0.99 0.99]; alpha = 0.8; %opaque
            end

            fill(ellipseCenter(1) + rotated_ellipse(1,:), ...
                 ellipseCenter(2) + rotated_ellipse(2,:), ...
                 color,'edgeColor',color,'FaceAlpha',alpha);
        end

        % Legacy code (deprecated on Dec 12 '25 - Khalid M Jaffar)
        % %3D - (x,y) + time - counterparts of the former functions
        % %draws the funnel with time-trajectory and ellipsoids information at the knot points
        % function drawFunnelwithTime(obj,funnel)
        %     %figure(2)
        %     N = length(funnel.trajectory_stateSpace);
        %     for j=N:-1:1
        %         P = funnel.invariantSet_stateSpace(:,:,j);
        %         xt = funnel.trajectory_stateSpace(:,j);
        %         t = funnel.time(:,j);
        %         drawEllipsewithTime(obj,t,xt,P);
        %     end
        %     drawTrajectorywithTime(obj,funnel); 
        % end
        % 
        % %draws trajectory with time in a 3D plot
        % function drawTrajectorywithTime(obj,funnel)
        %     %figure(2)
        %     traj = funnel.trajectory_stateSpace';
        %     plot3(traj(:,1),traj(:,2),obj.time,'--k','LineWidth',1.5);
        %     %Plotting the end and start points
        %     plot3(traj(1,1),traj(1,2),obj.time(1),'ok','LineWidth',1.5,'MarkerSize',2);
        %     plot3(traj(end,1),traj(end,2),obj.time(end),'.k','LineWidth',2,'MarkerSize',4);
        % end
        % 
        % %draws 2D ellipses "elevated" along the time axis
        % %status - 2 - goal branch; 1 - normal edge; 0 - deleted edge
        % function drawEllipsewithTime(obj,time,center,RofA,status)
        %     %figure(2);
        %     hold on
        %     N = 50;
        %     th = linspace(-pi,pi,N);
        %     Basis = [1 0; 0 1; 0 0; 0 0; 0 0; 0 0]; %xy
        %     %Basis = [0 0; 0 1; 0 1; 0 0; 0 0; 0 0]; %yz
        %     %Basis = [1 0; 0 0; 0 1; 0 0; 0 0; 0 0];  %zx
        %     E = Basis'/RofA*Basis;
        %     ell = E^(1/2)*[cos(th); sin(th)];
        %     time = time*ones(length(ell),1);
        % 
        %     if status == 2
        %         color = [0 0.9 0.1]; 
        %     elseif status == 1
        %         color = [0.8 0.8 0.8];
        %     else
        %         color = [1 1 1];
        %     end
        % 
        %     fill3(center(1) + ell(1,:),center(2) + ell(2,:),time, color,'edgeColor',color,'FaceAlpha',0.1);
        % end   
        % 
        % %draws the goal branch along the time axis as well
        % function drawGoalBranchwithTime(obj,searchGraph)
        %     %figure(2)
        % 
        %     %Constructing the goal branch by backtracking through parent pointers
        %     start = searchGraph.startNode.pose;
        %     goal = searchGraph.goalNode.pose;
        %     delta = 0.001;
        % 
        %     %Constructing the goal branch by backtracking through parent pointers
        %     temp = searchGraph.startNode;
        %     finishTime = temp.timeToGoal;
        % 
        %     while 1
        %         if abs(temp.pose - goal) < delta
        %             break
        %         end
        % 
        %         if (isnan(temp.parent))
        %             disp('There does not exist a feasible path currently!');
        %             return
        %             %break
        %         end
        % 
        %         tempFunnel = obj.funnelEdges(temp.bestInlet);
        %         x = tempFunnel.trajectory_stateSpace;
        %         t = tempFunnel.time;
        % 
        %         drawFunnelWithTime(obj,tempFunnel,2);
        %         plot3(x(:,1),x(:,2),t,'-.m','LineWidth',3);
        % 
        %         temp = searchGraph.graphNodes(temp.parent);
        % 
        %     end
        % 
        %     plot3(start(1),start(2),finishTime,'sg','LineWidth',3,'MarkerSize',7);
        %     plot3(goal(1),goal(2),0,'xr','LineWidth', 3,'MarkerSize',7);
        % end
        
        % Legacy code (deprecated on Dec 12 '25 - Khalid M Jaffar)
        %
        % function check = inFunnel(obj, config)
        % 
        %     check = 0;
        % 
        %     for i=1:obj.numFunnelEdges
        %         thisFunnel = obj.funnelEdges(i);
        %         trajectoryProjected = thisFunnel.trajectory_workSpace;
        %         invariantSetsProjected = thisFunnel.invariantSet_workSpace;        
        % 
        %         if obj.notInBoudingCircle(trajectoryProjected, config)
        %             continue
        %         end 
        % 
        %         N = length(thisFunnel.time);
        %         vanDerSequence = ceil(vdcorput(obj,N,2)*N);
        % 
        %         for k = 1:N
        %             nomStateProjected = trajectoryProjected(:,vanDerSequence(k));
        %             ellipsoidProjected = invariantSetsProjected(:,:,vanDerSequence(k));
        %             if(inBasin(obj,nomStateProjected,ellipsoidProjected,config))
        %                 check = 1;        
        %                 return
        %             end
        %         end
        %     end
        % end
        %
        % %checks whether funnel1 is composable with funnel2
        % %that is if outlet of funnel1 is contained within the inlet of funnel2
        % function check = isComposable(obj,funnel1,funnel2)
        % 
        %     %check = 1;  
        % 
        %     %outletRofA = funnel1.invariantSet_stateSpace(:,:,end); %index 'end': outlet of funnel 1
        %     outletCenter = funnel1.trajectory_stateSpace(:,end); %index 'end': outlet of funnel 1
        % 
        %     inletRofA = funnel2.invariantSet_stateSpace(:,:,1); %index 1: inlet of funnel 2
        %     inletCenter = funnel2.trajectory_stateSpace(:,1); %index 1: inlet of funnel 2
        % 
        %     %first pass check
        %     if(outletCenter-inletCenter)'*inletRofA*(outletCenter-inletCenter)>1 %if the centre itself doesn't lie in the ellipse, return  
        %         check = 0;
        %         %disp('Out in the first pass itself! (centre doesnot lie inside)')
        %         return
        %     end
        % 
        %     checkPoints = obj.decomposeOutletIntoEllipses(funnel1);
        % 
        %     check = obj.ellipsoidinEllipsoidCheck(inletCenter,inletRofA,checkPoints);
        % end
        %
        %-------------------------------------------------------------------------%
        % %Ellipsoid decomposition functions   
        % function checkPoints = decomposeOutletIntoEllipses(obj,funnel)
        % 
        % 
        %     checkResolution = 13; %keep it as an odd number preferably
        %     th = linspace(-pi,pi,checkResolution);
        % 
        %     numProjections = obj.stateDimension*(obj.stateDimension-1)/2;
        %     checkPoints = zeros(obj.stateDimension,checkResolution,numProjections); %2 because 2D ellipses
        % 
        %     %accessing the funnel's outlet properties
        %     %outletRofA = reshape(funnel.invariantSet_stateSpace(end,:,:),numDimensions,numDimensions);
        %     outletRofA = funnel.invariantSet_stateSpace(:,:,end);
        %     outletCenter = funnel.trajectory_stateSpace(:,end); %index 'end': outlet of funnel 1
        % 
        %     count = 1;
        % 
        %     for i = 1:obj.stateDimension-1
        %         for j = i+1:obj.stateDimension
        % 
        %             tempEllipsoid = inv(outletRofA);
        % 
        %             %accessing the corresponding elements
        %             E(1,1) = tempEllipsoid(i,i); E(1,2) = tempEllipsoid(i,j);
        %             E(2,1) = tempEllipsoid(j,i); E(2,2) = tempEllipsoid(j,j);
        % 
        %             %Getting the checkpoints on the boundary of the ellipse
        %             ell = E^(1/2)*[cos(th); sin(th)];
        %             %ell = sqrtm(E)*[cos(th); sin(th)];
        % 
        %             checkPoints(:,:,count) = outletCenter * ones(1,checkResolution);
        %             checkPoints(i,:,count) = ell(1,:) + checkPoints(i,:,count); %shifting origin
        %             checkPoints(j,:,count) = ell(2,:) + checkPoints(j,:,count); %shifting origin
        % 
        %             count = count+1; %keeping track of number of projections
        %         end
        %     end
        % end
        %
        % function check = ellipsoidinEllipsoidCheck(obj,inletCenter,inletRofA,checkPoints)
        % 
        %     check = 1;
        % 
        %     numDimensions = size(checkPoints,1);
        %     checkResolution = size(checkPoints,2);
        %     numProjections = size(checkPoints,3);
        % 
        %     %reshaping the matrix for ease of use
        %     checkPoints = reshape(checkPoints,numDimensions,checkResolution*numProjections);
        % 
        %     for i=1:size(checkPoints,2)
        %         thisCheckPoint = checkPoints(:,i);
        % 
        %         if(thisCheckPoint-inletCenter)'*inletRofA*(thisCheckPoint-inletCenter) > 1.1 %some extra allowance to account for numerical errors
        %             check=0;
        %             %(thisCheckPoint-inletCenter)'*inletRofA*(thisCheckPoint-inletCenter)
        %             return
        %         end
        %     end
        %    
        % end
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
        
     
        %Function to perform a course-check whether a configuration is not in the
        %bounding circle of the funnel, returns 1 if point is outside the circle
        function pass = notInBoudingCircle(obj,traj,config)
            pass = 0;
            initialConfig  = traj(:,1);
            finalConfig = traj(:,end);
            midConfig = (initialConfig+finalConfig)/2; %computing the approx centre of the trajectory

            radius = 1.1*euclidianDist(obj,initialConfig,finalConfig)/2;
            if(euclidianDist(obj,midConfig,config)>radius)
                pass = 1;
            end
        end
        
        %Function to determine whether a point lies inside an ellipse or not
        % (x'-x_c)'*Ellipsoid*(x'-x_c) < 1 implies x is within ellipsoid 
        function check = inBasin(obj,x_c, Ellipsoid, x)
            check = 0;
            
            if (x'-x_c)'*Ellipsoid*(x'-x_c) < 1
                check=1;
                return
            end
        end

        % -------------------------------------------------------%
        % new functions added (24 Oct 2025)
        % -------------------------------------------------------%

        % Function to check ellipsoid containment
        % checks whether ellipsoid 2 (red) is within ellipsoid 1 (blue)
        % or alternatively whether ellipsoid 1 (blue) contains ellipsoid 2 (red)
        function check = isComposable_usingSDP(obj, M_1, xc_1, M_2, xc_2)
            
            % Computes matrices for ellipsoid 1 (F, g, h)
            [F_1, g_1, h_1] = obj.generate_ellipsoid_params(M_1, xc_1);
        
            % Computes matrices for ellipsoid 2 (F, g, h)
            [F_2, g_2, h_2] = obj.generate_ellipsoid_params(M_2, xc_2);
        
            % Construct the LHS and RHS matrices for the SDP
            LHS = [F_1, g_1; g_1', h_1];  % LHS matrix 
            RHS = [F_2, g_2; g_2', h_2];  % RHS matrix (scaled by lambda)
            
            % SDP setup using SeDuMi or Mosek
            % Variables: lambda > 0
            cvx_begin sdp quiet
                cvx_solver mosek     % Use SeDuMi, or replace with cvx_solver mosek for Mosek
                variable lambda(1) nonnegative;  % Decision variable (lambda > 0)
                % Matrix inequality: LHS <= lambda * RHS
                LHS <= lambda * RHS;
            cvx_end
        
            % Output the results
            if strcmp(cvx_status, 'Solved')
                %fprintf('Ellipsoid containment is satisfied with lambda = %.4f\n', lambda);
                check = 1;
            else
                %fprintf('Ellipsoid containment is NOT satisfied.\n');
                check = 0;
            end
        end

        function [F, g, h] = generate_ellipsoid_params(obj, M, x_c)
            % Generates the parameters F, g, and h for an ellipsoid representation
            % in the form x'Fx + 2g'x + h <= 0 given the matrix M and center x_c
            % from the center-form representation (x-x_c)'M(x-x_c) < 1
            
            F = M;
            g = -M*x_c;
            h = x_c'*M*x_c - 1;
        end

        function check = isComposable_usingSurfaceSampling(obj, inletRofA, inletCenter, outletRofA, outletCenter, numSamplePoints)
            
            outletEllipsoidSurfacePoints = obj.samplePointsOnEllipsoidSurface(outletCenter,outletRofA, numSamplePoints);
            
            check = obj.ellipsoidinEllipsoidCheck(inletCenter,inletRofA,outletEllipsoidSurfacePoints);
        end        
        
        % Ellipsoid decomposition functions   
        function ellipsoid_surface_points = samplePointsOnEllipsoidSurface(obj, outletCenter,outletRofA,numSamplePoints)
            % Eigen decomposition
            [Q, Lambda] = eig(outletRofA);
            n = size(outletCenter,1);
        
            % Semi-axis lengths
            semi_axes_lengths = 1 ./ sqrt(diag(Lambda));
            
            % Generate random points on the n-dimensional unit sphere
            sphere_points = randn(n, numSamplePoints); % Random points
            sphere_points = sphere_points ./ vecnorm(sphere_points); % Normalize to lie on the unit sphere
            
            % Transform points to the ellipsoid
            ellipsoid_surface_points = Q * diag(semi_axes_lengths) * sphere_points + outletCenter;
        end
        
        function check = ellipsoidinEllipsoidCheck(obj, inletCenter,inletRofA,checkPoints)
        
            check = 1;
            
            %checkpoints array of dimension -- n * numSamples
            for i=1:size(checkPoints,2)
                thisCheckPoint = checkPoints(:,i);
                
                if(thisCheckPoint-inletCenter)'*inletRofA*(thisCheckPoint-inletCenter) > 1
                    check=0;
                    plot3(thisCheckPoint(1),thisCheckPoint(2),thisCheckPoint(3),'xg');
                    return
                end
            end
            
        end

        function [trajectory_mD, ellipsoids_mD] = projectFunnel_nD_to_mD(obj, funnelEdge, projection_dims)
            % Input:
            % funnelEdge 
            % projection_dims: m-element vector specifying which dimensions to project onto
            %                  (e.g., [1 2] for xy-plane, [1 2 3] for xyz-plane, etc.)
            
            trajectory_nD = funnelEdge.trajectory_stateSpace; %n*N trajectory
            ellipsoids_nD = funnelEdge.invariantSet_stateSpace; %n*n*N ellipsoid matrices 

            %get the dimensionality of state-space and projection space
            n = size(trajectory_nD, 1);     
            m = length(projection_dims); 
            
            %construct the basis matrix
            basisMatrix = zeros(n,m);
            for i=1:m
                basisMatrix(projection_dims(i),i) = 1;
            end

            N = size(trajectory_nD, 2); %number of time instances 

            ellipsoids_mD = NaN(m,m,N);
            trajectory_mD = NaN(m,N);

            for k = 1:N
                temp_trajectory_nD = trajectory_nD(:,k);
                temp_ellipsoid_nD = ellipsoids_nD(:,:,k);
        
                ellipsoids_mD(:,:,k) = inv(basisMatrix' / temp_ellipsoid_nD * basisMatrix);
                trajectory_mD(:,k) = basisMatrix' * temp_trajectory_nD;
            end
        end

    end
end

