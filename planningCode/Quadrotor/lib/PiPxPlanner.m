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
        extendDistance
        resolution
        CspaceDimensionality
        initial_rBall_radius
        drawFlag
        currentRobotState
        startConfig
        goalConfig
        useNewColorForPlotting %new color scheme for pretty plotting
    end %end of properties

    methods
        function obj = PiPxPlanner(envLB,envUB,epsilon,resolution,drawFlag) %constructor class
            
            obj.envLB = envLB;
            obj.envUB = envUB;
            obj.extendDistance = epsilon;
            obj.resolution = resolution;
            obj.CspaceDimensionality = 2; %xy-planning
            obj.initial_rBall_radius = 50; %used for the shrinking rBall radius compute

            if nargin < 5
                obj.drawFlag = 0;
            else
                obj.drawFlag = drawFlag;
            end           

            obj.useNewColorForPlotting = true; %true for paper plotting

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

        function flag = generateFunnelRRG(obj,F,C,G,W,T,startFound,robotMove,epsilon)

            newNodePose = C.expandSearchGraph(T,W,startFound,robotMove,epsilon,F.resolution);
            
            %plot(newNodePose(1),newNodePose(2), 'xb','MarkerSize',7,'LineWidth',1.4)
            %drawnow
            
            if(~W.vertexCollisionFree(newNodePose) || F.inAnyInlets(newNodePose))
                flag = 1; return
            end
            
            %Find the neighbors within an r-Ball
            [neighbors, flag] = obj.findNeighborsInRBall(T,newNodePose);
         
            if flag %if no neighbors found, continue with the next sampling
                return
            end
        
            thisNode = nodeStruct(C.numNodes+1,newNodePose);

            %add the the new sampled node to existing funnel-network 
            flag = F.constructFunnelNetwork(T,C,W,thisNode,neighbors);
            
            if flag %if no new edges were added, continue with the next sampling
                return
            end
            
            G.constructAugmentedGraph(F,C,thisNode);
        end
        
        function flag = addStartNodeToFunnelRRG(obj,F,C,G,W,T,startPose)
        
            %nearestNeighbor = C.graphNodes(end).pose; %using the neighbors of the previous node
            %[neighbors, flag] = obj.findNeighborsInRBall(T,nearestNeighbor);
            
            [neighbors, flag] = obj.findNeighborsInRBall(T,startPose);

            if flag %if no neighbors found, exit
                return
            end
            
            startNode = nodeStruct(C.numNodes+1,startPose);
            flag = F.constructFunnelNetwork(T,C,W,startNode,neighbors);
            
            if flag %if no funnel-edges were added, exit
                return
            end
            
            G.constructAugmentedGraph(F,C,startNode);
            C.startNode = startNode; F.startNode = startNode;
        end
        
        function [status, traversingFunnelEdge] = moveRobot(obj,F,C,G,Q)
            
            if obj.goalCheck(obj.currentRobotState)
                status = 1; %if already at goal region break out
                traversingFunnelEdge = NaN;
                return
            end
        
            %save robot's previous node information
            G.updatekm(C); %needs to be updated only when robot moves
            C.previousRobotNode = C.startNode;   
            
            %computes the solution path
            status = C.findBestInletAtStartNode(G,F,Q);
            
            if ~status
                traversingFunnelEdge = NaN;
                return
            end

            C.findParentInletsAtEachNode(G);
            F.constructShortestFunnelPath(G); 
            
            traversingFunnelEdge = F.funnelEdges(C.currentRobotNode.parentFunnelEdge);

            if obj.drawFlag
                if obj.useNewColorForPlotting
                    colorScheme = newColorScheme();
                    F.drawFunnel(traversingFunnelEdge,2);
                    x = traversingFunnelEdge.trajectory_workSpace;
                    plot(x(1,:), x(2,:), '-', 'Color', colorScheme.branchTraj, 'LineWidth', 0.8);
                else
                    F.drawFunnel(traversingFunnelEdge,2);
                end
            end
            
            %move the robot by applying the closed-loop control commands
            %two options for dynamics integrator: 'trapezoidal' [preferred], 'rk4'
            newRobotState = obj.simulateClosedLoopDynamics(traversingFunnelEdge, 'trapezoidal');
            obj.currentRobotState = newRobotState;

            %update node pointers in the underlying graph
            parentIndex = C.startNode.parent;
            C.startNode = C.graphNodes(parentIndex); F.startNode = C.startNode;
            C.currentRobotNode = C.startNode;
            C.currentRobotNode.pose = obj.currentRobotState(1:2)';
            
            % norm(obj.currentRobotState(F.workspaceDimensionIndices) - C.currentRobotNode.pose')
        end

        function newState = simulateClosedLoopDynamics(obj, funnelEdge, integrator)
            % Propagate the closed-loop quadrotor along the funnel edge.
            % Control law: u = u_nom - K (x - x_nom), with u_nom held constant over
            % each interval (ZOH), as in the collocation transcription.
            % integrator: 2 options
            % 1.'trapezoidal' (default) - implicit trapezoidal rule with feedback evaluated
            %      at both interval ends. A-stable, and reduces to the collocation
            %      constraint on the nominal, so a robot starting on the nominal
            %      reproduces the nominal knot states.
            % 2.'rk4' - classical RK4 with feedback re-evaluated at every stage against
            %      the collocation interpolant of the nominal. May not reproduce the
            %      knots exactly (transcription mismatch).
            
            if nargin < 3, integrator = 'trapezoidal'; end
            integrator = validatestring(integrator, {'trapezoidal', 'rk4'});
        
            x_nom = funnelEdge.trajectory_stateSpace;
            u_nom = funnelEdge.feedforwardControlInputs;
            K     = funnelEdge.feedbackControlGains;
            E     = funnelEdge.invariantSet_stateSpace;
            t     = funnelEdge.time;
            N     = size(x_nom, 2);
            f     = @(x, u) systemDynamics(x, u);
        
            currState = obj.currentRobotState(:);
            % Checking initial containment: for funnel invariance guarantees to be valid
            startContained = funnelCheck(currState, x_nom(:,1), E(:,:,1));
            if ~startContained
                error('The current robot state doesnot lie in the inlet of the traversing funnel-edge');
            end

            for k = 1:N-1
                dt = t(k+1) - t(k);
                switch integrator
                    case 'trapezoidal'
                        currState = trapezoidalCLStep(f, currState, x_nom(:,k), x_nom(:,k+1), u_nom(:,k), K(:,:,k), K(:,:,k+1), dt);
                    case 'rk4'
                        currState = rk4FeedbackStep(f, currState, x_nom(:,k), x_nom(:,k+1), u_nom(:,k), K(:,:,k), dt);
                end
            end
        
            % Terminal set containment: required for composability with the child edge
            terminalContained = funnelCheck(currState, x_nom(:,N), E(:,:,N));
            if ~terminalContained
                warning('The final robot state didnot terminate inside the outlet: build-up of numerical integration errors');
            end
        
            newState = currState;
        end
 
        %-------------------------------------------------------------------------%
        %Environment dynamicity (as sensed by the robot) related function
        function makeDynamicChangesToGraph(obj,F,C,G,Q,W,T,varargin)   
        
            if strcmpi(W.mode, 'sensing') 
                exploredObstacles = W.senseObstacles(C.currentRobotNode.pose); %sense from the middle
                modifiedEdges = W.getModifiedEdges(F,C,G,T,exploredObstacles);
            elseif strcmpi(W.environmentType, 'forest') && strcmpi(W.mode, 'dynamic') 
                numChangedObstacles = ceil(W.numObstacles * W.dynamicity/100);

                if numChangedObstacles == 0
                    modifiedEdges = [];
                else
                    %deletion
                    deletedObstacles = W.removeRandomObstacles(F,G,numChangedObstacles);
                    freedUpEdges = W.getModifiedEdges(F,C,G,T,deletedObstacles,'deletion');
                    if obj.drawFlag
                        W.drawAllObstacles();
                        drawnow;
                    end
                    %addition
                    addedObstacles = W.addRandomObstacles(C.currentRobotNode.pose, C.goalNode.pose, numChangedObstacles);
                    newCollisionEdges = W.getModifiedEdges(F,C,G,T,addedObstacles,'addition');
                    
                    modifiedEdges = [freedUpEdges, newCollisionEdges];
                end
            elseif strcmpi(W.environmentType, 'forest') && strcmpi(W.mode, 'moving') 
                numChangedObstacles = ceil(W.numObstacles * W.dynamicity/100);

                if numChangedObstacles == 0
                    modifiedEdges = [];
                else
                    %deletion
                    deletedObstacles = W.removeRandomObstacles(F,G,numChangedObstacles);
                    freedUpEdges = W.getModifiedEdges(F,C,G,T,deletedObstacles,'deletion');
                    if obj.drawFlag
                        W.drawAllObstacles();
                        drawnow;
                    end
                    %addition
                    addedObstacles = W.addShiftedObstacles(deletedObstacles,C.currentRobotNode.pose, C.goalNode.pose);
                    newCollisionEdges = W.getModifiedEdges(F,C,G,T,addedObstacles,'addition');
                    
                    modifiedEdges = [freedUpEdges, newCollisionEdges];
                end
            elseif strcmpi(W.environmentType, 'maze') && strcmpi(W.mode, 'dynamic')
                
                %sense maze walls from the robot location
                exploredObstacles = W.senseObstacles(C.currentRobotNode.pose);
                modifiedEdges = W.getModifiedEdges(F,C,G,T,exploredObstacles);
                
                %open and close windows in the maze walls at random
                numChangedWindows = round(length(W.sensedWindows) * W.dynamicity/100);
                if numChangedWindows ~= 0
                    %deletion - open the windows
                    if strcmpi(varargin(1), 'windows-open') 
                        deletedObstacles = W.removeRandomObstacles(F,G,numChangedWindows);
                        freedUpEdges = W.getModifiedEdges(F,C,G,T,deletedObstacles,'deletion');
                        newCollisionEdges = [];
                    elseif strcmpi(varargin(1), 'windows-close') 
                        %addition - close windows
                        addedObstacles = W.addRandomObstacles(C.currentRobotNode.pose, C.goalNode.pose, numChangedWindows);
                        newCollisionEdges = W.getModifiedEdges(F,C,G,T,addedObstacles,'addition');
                        freedUpEdges = [];
                    else
                        newCollisionEdges = [];
                        freedUpEdges = [];
                    end
                    
                    %will have to remove duplicates
                    modifiedEdges = [modifiedEdges, freedUpEdges, newCollisionEdges];
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
                neighbors = nodeStruct();
                flag = 1;
                return
            end
        end
        
        %Shrinking r-Ball
        function r = rBall(obj,iteration)
            
            %Shrinking rate from RRT* paper
            r0 = obj.initial_rBall_radius; iteration = iteration+1;
            %Converting infinity norm to 2-norm (based on pre-computed funnel library)
            epsilon = obj.extendDistance * obj.CspaceDimensionality^(1/obj.CspaceDimensionality); %L_infinity-norm to L2-norm
            %epsilon = obj.extendDistance;

            r = min(r0*(log(iteration)/(iteration))^(1/obj.CspaceDimensionality), epsilon);
            r = max(r, 1/obj.resolution);
            
            %rBall radius is saturated by max extend distance (UB) and
            %resolution of the funnelLibrary (LB)
        end
        
        %Goal checking
        function success = goalCheck(obj,robotState)
            success = norm(robotState(1:3)-obj.goalConfig') < obj.resolution + 1e-6; %goal region radius = 1 (library resolution)
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

            %draw goal region
            r = obj.resolution; theta = linspace(0, 2*pi, 100);
            x = obj.goalConfig(1) + r * cos(theta); y = obj.goalConfig(2) + r * sin(theta);
            fill(x, y, [0.84 0.37 0.00], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
            plot(obj.goalConfig(1), obj.goalConfig(2), 'x', 'Color', [0.84 0.37 0.00], ...
                'MarkerSize', 4, 'LineWidth', 1.5);
        end
        
        % Fixed 1x3 layout for preplanning figure, full text width
        function [fig, ax] = makePreplanFig(obj)
            W = 6.85;  s = 2.00;                           % figure width, panel side (in)
            mL = 0.30;  gap = 0.25;  mB = 0.25;  mT = 0.20; % margins; gap holds next panel's y ticks
            H = mB + s + mT;                               % = 2.45 in
            fig = ijrr_fig('double');
            set(fig, 'Position', [1 1 W H], 'PaperSize', [W H], 'Resize', 'off', ...
                'PaperPositionMode', 'manual', 'PaperPosition', [0 0 W H]);
            ax = gobjects(1, 3);
            for k = 1:3
                ax(k) = axes(fig, 'Units', 'inches', 'Position', [mL + (k-1)*(s + gap), mB, s, s], ...
                    'PositionConstraint', 'innerposition');
            end
        end

        function ax = preplanningPhasePlotting(obj,ax,C,G,F,W,totalIterationLimit)
            Q = heap(totalIterationLimit); %initialise the priority queue with the total iteration limit
            C.previousRobotNode = C.startNode; C.currentRobotNode = C.startNode;
            
            G.startVertex = G.graphVertices(C.startNode.inletVertices(1));
            
            %Graph-search initialisation
            %all nodes have infinite g and lmc value by default (constructor definition)
            G.initialiseGraphSearch(Q);
            
            %determine the best inlet to take at the start configuration
            %disp(' ');
            robotMoveStatus = C.findBestInletAtStartNode(G,F,Q);
            
            C.findParentInletsAtEachNode(G);
            F.constructShortestFunnelPath(G);

            % Plot all elements
            colorScheme = newColorScheme();

            %view(ax, -45, 45);                    % azimuth, elevation in degrees; close to your old figure
            axis(ax, 'equal'); % keep the plane undistorted
            ax.Box = 'off';                        % no 3D box edges
            xlim(ax, [W.envLB W.envUB])
            ylim(ax, [W.envLB W.envUB])
            %zlim(ax, [0 1e-3]);                    % essentially zero thickness
            %ax.ZAxis.Visible = 'off';              % no z-axis line, ticks or labels
            ax.XTick = [0 25 50];  ax.YTick = [0, 25 50]; ax.YTickLabel = {'', '25', '50'};  % same ticks in every panel

            % % Outline of the 50 x 50 workspace on the ground plane
            rectangle('Position',[W.envLB W.envLB W.envUB W.envUB]);
        
            F.drawSearchFunnel(); F.drawGoalBranch();
            W.drawAllObstacles();

            plot(C.currentRobotNode.pose(1), C.currentRobotNode.pose(2), 'd', 'MarkerFaceColor', colorScheme.robot, ...
                    'MarkerEdgeColor', 'k', 'MarkerSize', 4, 'LineWidth', 0.5);   % robot
            drawnow
        end

        % inset figures for magnigication
        function drawInsetScene(obj, a, F, W, region)
            % fig = ancestor(a, 'figure');
            % set(fig, 'CurrentAxes', a);       % class methods draw into gca
            fig_ = ancestor(a, 'figure');
            set(groot, 'CurrentFigure', fig_);
            set(fig_, 'CurrentAxes', a);
            hold(a, 'on');
            W.drawAllObstacles();             % same obstacles as the global view
            drawRoadmapDetail(a, F, region, EllipseEvery=1); %TreeEvery=Inf
        end

        function P = solutionNodesXY(obj, F)
            P = [];  node = F.startNode;  goal = F.goalNode.pose;
            for k = 1:F.numNodes
                P(end+1, :) = node.pose(1:2);
                if all(abs(node.pose - goal) < 0.01) || isnan(node.parent), break; end
                node = F.graphNodes(node.parent);
            end
        end

        %-------------------------------------------------------------------------%
        %Saving data functions
        function fileCount = saveData(obj,F,C,G,W,dir,fileCount)
            funnelNetwork = F;
            configurations = C;
            augmentedGraph = G;
            workspace = W;
            save([dir 'iteration_' num2str(fileCount) '.mat'],'funnelNetwork','configurations','augmentedGraph','workspace');
            fileCount = fileCount+1;
        end
    
    end %end of methods
    
end %end of class defintion

%% Local helper functions
function y = trapezoidalCLStep(f, x, x_k, x_k1, u_nom, K_k, K_k1, h)
    % Implicit trapezoidal step with feedback evaluated at both interval ends.
    % Reduces to the collocation constraint when x is on the nominal.
    n  = numel(x);
    fx = f(x, u_nom - K_k*(x - x_k));
    g  = @(y) f(y, u_nom - K_k1*(y - x_k1));
    y  = x_k1 + (x - x_k); % predictor: nominal + current deviation
    for it = 1:20
        gy = g(y);
        r  = y - x - (h/2)*(fx + gy);
        if norm(r, inf) <= 1e-12*(1 + norm(y, inf)), return; end
        J = zeros(n);
        for i = 1:n
            d = 1e-7*max(1, abs(y(i)));
            e = zeros(n,1);  e(i) = d;
            J(:,i) = (g(y + e) - gy)/d;
        end
        y = y - (eye(n) - (h/2)*J) \ r;
        if any(~isfinite(y)), error('PiPX:divergence', 'State diverged (non-finite).'); end
    end
    warning('PiPX:TrapezoidalNewton', 'Trapezoidal step did not converge (residual %.2e).', norm(r, inf));
end

function x = rk4FeedbackStep(f, x, x_k, x_k1, u_nom, K_k, dt)
    % RK4 with feedback re-evaluated at every stage against the trapezoidal-
    % collocation interpolant of the nominal (u_nom held over the interval)
    numSteps = 50;  % finer integration step size to prevent numerical errors

    f_k  = f(x_k,  u_nom);
    f_k1 = f(x_k1, u_nom);
    x_nom = @(tau) x_k + f_k*tau + (f_k1 - f_k)*tau^2/(2*dt);
    g     = @(tau, x) f(x, u_nom - K_k*(x - x_nom(tau)));

    dt_fine = dt/numSteps;  tau = 0; % finer integration to prevent numerical error
    for i = 1:numSteps
        k1 = g(tau,        x);
        k2 = g(tau + dt_fine/2, x + dt_fine/2*k1);
        k3 = g(tau + dt_fine/2, x + dt_fine/2*k2);
        k4 = g(tau + dt_fine,   x + dt_fine*k3);
        x  = x + dt_fine/6*(k1 + 2*k2 + 2*k3 + k4);
        tau = tau + dt_fine;
    end
end

function inside = funnelCheck(x, x_nom, E)
    tolerance = 1e-6;
    xbar     = x - x_nom;
    level  = xbar.' * E * xbar;
    inside = level <= 1 + tolerance;
end