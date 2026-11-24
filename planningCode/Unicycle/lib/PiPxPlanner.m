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
        currentRobotState
        startConfig
        goalConfig
        useNewColorForPlotting %new color scheme for pretty plotting
    end %end of properties

    methods
        function obj = PiPxPlanner(envLB_x,envUB_x,envLB_y,envUB_y,extendDistance,resolution,drawFlag) %constructor class
            
            obj.envLB_x = envLB_x - 0.5; obj.envUB_x = envUB_x + 0.5; %adding/subtracting 0.5 to better visualize road-lanes
            obj.envLB_y = envLB_y; obj.envUB_y = envUB_y;
            obj.extendDistance = extendDistance;
            obj.resolution = resolution;
            obj.CspaceDimensionality = 2; %xy-planning
            obj.initial_rBall_radius = 50; %used for the shrinking rBall radius compute

            if nargin < 5
                obj.drawFlag = 0;
            else
                obj.drawFlag = drawFlag;
            end
    
            obj.useNewColorForPlotting = true; %true for paper plotting
        end

        function flag = generateFunnelRRG(obj,F,C,G,W,T,startFound,robotMove)
            
            %epsilon = obj.extendDistance*obj.CspaceDimensionality^(1/obj.CspaceDimensionality); %L2-norm of the extend distance (xy-planning)
            epsilon = obj.extendDistance; %using a conservative epsilon (extendDistance) for expanding the searchFunnel
                                          %note that although, in principle, we should be able to
                                          %use the previous line epsilon (upper bound value) as well
            newNodePose = C.expandSearchGraph(T,W,startFound,robotMove,epsilon,F.resolution);
            
            %plot(newNodePose(1),newNodePose(2), 'xb','MarkerSize',7,'LineWidth',1.4)
            %drawnow
            
            if(~W.vertexCollisionFree(newNodePose) || F.inAnyInlets(newNodePose))
                flag = 1; return
            end
            
            %Find the neighbors within an r-Ball
            [potentialNeighbors, flag] = obj.findNeighborsInRBall(T,newNodePose);
         
            if flag %if no neighbors found, continue with the next sampling
                return
            end
        
            thisNode = nodeStruct(C.numNodes+1,newNodePose);

            %add the the new sampled node to existing funnel-network 
            flag = F.constructFunnelNetwork(T,C,W,thisNode,potentialNeighbors);
            
            if flag %if no new edges were added, continue with the next sampling
                return
            end
            
            G.constructAugmentedGraph(F,C,thisNode);
        end
        
        function flag = addStartNodeToFunnelRRG(obj,F,C,G,W,T,startPose)
        
            %nearestNeighbor = C.graphNodes(end).pose; %using the neighbors of the previous node
            %[neighbors, flag] = obj.findNeighborsInRBall(T,nearestNeighbor);
            
            [potentialNeighbors, flag] = obj.findNeighborsInRBall(T,startPose);

            if flag %if no neighbors found, exit
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
            %two options for dynamics integrator: 'trapezoidal' (preferred) [default], 'rk4'
            newRobotState = obj.simulateClosedLoopDynamics(traversingFunnelEdge);
            obj.currentRobotState = newRobotState;

            %update node pointers in the underlying graph
            parentIndex = C.startNode.parent;
            C.startNode = C.graphNodes(parentIndex); F.startNode = C.startNode;
            C.currentRobotNode = C.startNode;

            % norm(obj.currentRobotState(F.workspaceDimensionIndices) - C.currentRobotNode.pose')

            %update pose information of robot node in the graph
            C.currentRobotNode.pose = obj.currentRobotState(1:2)';
        end
        
        function newState = simulateClosedLoopDynamics(obj, funnelEdge, integrator)
            % Propagate the closed-loop system dynamics along the funnel edge.
            % Control law: u = u_nom - K (x - x_nom), with u_nom held constant over
            % each interval (ZOH), as in the collocation transcription.
            % integrator: 2 options
            % 1.'rk4' - classical RK4 with feedback re-evaluated at every stage against
            %      the collocation interpolant of the nominal. May not reproduce the
            %      knots exactly (transcription mismatch).
            % 2.'trapezoidal' [default] - implicit trapezoidal rule with feedback input evaluated
            %      at both interval ends. A-stable, and reduces to the collocation
            %      constraint on the nominal, so a robot starting on the nominal
            %      reproduces the nominal knot states.
            
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
                    case 'rk4'
                        currState = rk4Integration(f, currState, x_nom(:,k), x_nom(:,k+1), u_nom(:,k), K(:,:,k), dt);
                    case 'trapezoidal'
                        currState = trapezoidalIntegration(f, currState, x_nom(:,k), x_nom(:,k+1), u_nom(:,k), K(:,:,k), K(:,:,k+1), dt);
                end
            end
            newState = currState;

            % Terminal set containment: required for composability with the child edge
            terminalContained = funnelCheck(newState, x_nom(:,N), E(:,:,N));
            if ~terminalContained
                warning('The final robot state didnot terminate inside the outlet: build-up of numerical integration errors!');
            end

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
                    %addition
                    addedObstacles = W.addRandomObstacles(C.currentRobotNode.pose, C.goalNode.pose, numChangedObstacles);
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
            epsilon = obj.extendDistance * obj.CspaceDimensionality^(1/obj.CspaceDimensionality); %L_infinity-norm to L2-norm conversion
            
            r = min(r0*(log(iteration)/(iteration))^(1/obj.CspaceDimensionality), epsilon); %upper saturation
            r = max(r, max(obj.resolution)); %max or min           %lower saturation
            
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
            figure; clf;
            axis equal
            xlim([obj.envLB_x obj.envUB_x])
            ylim([obj.envLB_y obj.envUB_y])
            hold on
            rectangle('Position',[obj.envLB_x, obj.envLB_y, obj.envUB_x - obj.envLB_x, obj.envUB_y - obj.envLB_y]) %lower-left corner, width, height
        end
        
        %-------------------------------------------------------------------------%
        %Saving data functions
        function fileCount = saveData(obj,F,C,G,W,dir,fileCount)
            % configurations = C.graphNodes;
            % %edges = C.graphEdges;
            % graphVertices = G.graphVertices;
            % graphEdges = G.graphEdges;
            % funnels = F.funnelEdges;  
            % obstacles = W.obstacles;
            % robotNode = C.startNode;
            % save([dir 'iteration_' num2str(fileCount) '.mat'],'configurations','graphVertices','graphEdges','funnels','obstacles','robotNode');
            
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

function nextState = rk4Integration(f, x_k, x_nom_k, x_nom_k1, u_nom_k, K_k, dt)
    % RK4 with closed-loop dynamics with finer integration step, using a
    % trapezoidal-based (quadratic) interpolant of the nominal trajectory. 
    % u_nom and feedback gain, K_k held constant over the interval (ZOH)

    numSteps = 50; dt_fine = dt/numSteps; % finer integration step size to prevent numerical errors

    f_nom_k  = f(x_nom_k,  u_nom_k);
    f_nom_k1 = f(x_nom_k1, u_nom_k);
    x_nom_interp = @(tau) x_nom_k + f_nom_k*tau + (f_nom_k1 - f_nom_k)*tau^2/(2*dt);
    g    = @(tau, x) f(x, u_nom_k - K_k*(x - x_nom_interp(tau))); %closed-loop dynamics function

    tau = 0; % finer integration to prevent numerical integration errors
    for i = 1:numSteps
        k1 = g(tau,        x_k);
        k2 = g(tau + dt_fine/2, x_k + dt_fine/2*k1);
        k3 = g(tau + dt_fine/2, x_k + dt_fine/2*k2);
        k4 = g(tau + dt_fine,   x_k + dt_fine*k3);
        x_k  = x_k + dt_fine/6*(k1 + 2*k2 + 2*k3 + k4);

        tau = tau + dt_fine;
    end

    nextState = x_k;
end

function nextState = trapezoidalIntegration(f, x_k, x_nom_k, x_nom_k1, u_nom_k, K_k, K_k1, dt)
    % Implicit trapezoidal integration of the closed-loop dynamics over [t_k, t_{k+1}].
    % Feedforward input is held over the interval (as in the collocation transcription), 
    % feedback control input is evaluated at both interval ends.
    % x_{k+1} = x_k + (dt/2) (g_k(x_k) + g_{k+1}(x_{k+1})),
    % closed-loop dynamics: g_j(x)  = f(x, u_nom_k - K_j (x - x_nom_j)).

    n = numel(x_k);
    g_at_currState = f(x_k, u_nom_k - K_k*(x_k - x_nom_k));   % closed loop dynamics at t_k
    g = @(x) f(x, u_nom_k - K_k1*(x - x_nom_k1));   % fn handle for closed loop dynamics at t_{k+1}

    % Initial guess for x_{k+1} for the iterative Newton-solve (solve residual r(y) = 0)
    nextState = x_nom_k1 + (x_k - x_nom_k); %next nominal knot + current deviation (for faster convergence)
    for iter = 1:20
        g_at_nextState = g(nextState);
        residual = nextState - (x_k + (dt/2)*(g_at_currState + g_at_nextState));

        % Mixed relative/absolute tolerance; keep below the collocation tolerance
        if norm(residual, inf) <= 1e-10*(1 + norm(nextState, inf)), return; end

        J = zeros(n); %numerical Jacobian computation, dg_{k+1}/dy (using forward differences)
        for i = 1:n
            delta = 1e-7*max(1, abs(nextState(i)));  % relative step size, floored at 1e-7 near zero
            perturbation = zeros(n,1);  perturbation(i) = delta;
            J(:,i) = (g(nextState + perturbation) - g_at_nextState)/delta;
        end

        % Newton update: y <- y <- y - (dr/dy)^{-1} r(y), with dr/dy = I - (dt/2) J
        nextState = nextState - (eye(n) - (dt/2)*J) \ residual;

        if any(~isfinite(nextState))
            warning('PiPX:divergence', 'State diverged (non-finite).');
            return;
        end
    end
    warning('PiPX:TrapezoidalNewton', 'Trapezoidal step did not converge (residual %.2e).', norm(residual, inf));
end

function inside = funnelCheck(x, x_nom, E)
    tolerance = 1e-6;
    xbar     = x - x_nom;
    level  = xbar.' * E * xbar;
    inside = level <= 1 + tolerance;
end