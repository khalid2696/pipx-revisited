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

clc
clearvars
close all

%adding paths to code libraries and funnel-library
addpath('./lib/');
addpath('./precomputedFunnelLibrary/');

%configurable flags
drawFlag = 1;
saveFlag = 0;
fileCount = 1; %for saving files in /temp/ folder

%Assigning values to algorithm parameters
epsilon = 5;          %extend-distance
prePlanningIterationLimit = 200; 
totalIterationLimit = 300; %Maximum number of iterations %keep it less than 300 always!
idleTimeLimit = 5;
planningFrequency = 1;
robotMovementFrequency = 3; %decreasing this parameter increases the robot speed!
sensingFrequency = robotMovementFrequency; %for this particular forest-sense planning problem
numTreeObstacles = 15;

envLB = 0;
envUB = 50;

planner = PiPxPlanner(envLB,envUB);
planner.setupPlot()

O = obstacleList(envLB,envUB,epsilon,1); %obstacle class  epsilon - tolerance
                                         %mode - 1 for sensing, 2 for
                                         %dynamic addition/deletion

%distFunct = @(inputA, inputB) sqrt(sum((inputA - inputB).^2,2)); %distance function (for kDTree)
T = KDTree(2, @(inputA, inputB) sqrt(sum((inputA - inputB).^2,2))); %initialise the tree, 2 - num of dimensions of configuration space

load('funnelLibraryNominal.mat');
F = searchFunnel(funnelLibrary,'nominal');
C = configurationSpace();  %instantiate an empty configuration space class
G = searchGraph(); %augmented graph data structure to store F and C

%------------------------------------%
%user-input start and goal locations
%------------------------------------%
waitfor(msgbox('Click on the start and goal positions respectively'));
[problemx,problemy] = ginput(2);

startPose = [problemx(1) problemy(1)];
goalPose = [problemx(2) problemy(2)];

%--------------------------------%
%random start and goal locations
%--------------------------------%
%startPose = rand([1 2])*(O.envUB-envLB) + O.envLB;
%goalPose = rand([1 2])*(O.envUB-envLB) + O.envLB;

%------------------------------------------------%
%fixed start and goal locations (for dev purposes)
%------------------------------------------------%
%startPose = [5.4,4.7];
%goalPose = [45.6,44.8];

%initially adding obstacles
O.addDynamicObstacles(numTreeObstacles,startPose,goalPose); %argin - #obstacles, robot pose, goal pose, 
                                                      
O.initialiseObstacleTree();
O.senseObstacles(startPose);

if(~O.vertexCollisionFree(goalPose))
    error('Goal inside the obstacles. No path exists!')
end

%progress variables
iteration = 1;            %keeps track of number of nodes in tree
startFound = 0;             %Start check
robotMove = 0;
idleTime = 0;

traversedPathLength = 0;
remainingPathLength = inf;

%Funnel-RRG (for configuration graph and visualisation)
goalNode = nodeStruct(1,goalPose);
goalNode.cost = 0;
C.addNode(goalNode); F.addNode(goalNode);

C.goalNode = goalNode; F.goalNode = goalNode;
C.startNode = nodeStruct(NaN,startPose); F.startNode = nodeStruct(NaN,startPose); %start node will be added later

%augmented graph data structure
goalVertex = augmentedVertexStruct(1, goalNode.index, NaN); %(id, configuration, funnel)
goalVertex.type = 'inlet'; %always the goal region is a "sink"
goalVertex.pose = goalNode.pose; goalVertex.cost = 0;
G.addVertex(goalVertex);
goalNode.inletVertices = goalVertex.index;

G.goalVertex = goalVertex; G.startVertex = augmentedVertexStruct(); %start vertex will be updated later

T.kdInsertAsPayload(goalNode);

%Sample class-constructor usage

%[nearestNode, distToClosestNode] = T.kdFindNearestPayload(queryPoint)
%neighborsInRange = T.kdFindWithinRangePayload(range, queryPoint)

%nodeStruct(id, [xPose yPose])
%edgeStruct(id, [parent child], cost) 
%searchGraph(nodes,edges)
%obstacleList(obstacles)

%save the initial environment
if saveFlag
    dir = ['./temp/trial' num2str(1) '/'];
    mkdir(dir);
    fileCount = planner.saveData(F,C,O,dir,fileCount);
end

%draw the initial environment
if drawFlag
    %Plotting start and goal positions
    plot(goalPose(1), goalPose(2), 'xr', 'MarkerSize', 8, 'LineWidth', 3.5)
    plot(startPose(1), startPose(2), 'sg', 'MarkerSize', 8, 'LineWidth', 3.5)
    O.drawAllObstacles(); O.drawSensorRadius(C.startNode.pose);
    drawnow
end

tic

%% -----------------------------------------------------------%
% Pre-planning phase of generating a roadmap of funnels
%-----------------------------------------------------------%
progressBar = waitbar(0, 'Funnel RRG construction progress');
while iteration < prePlanningIterationLimit %&& ~startFound
    
    flag = planner.generateFunnelRRG(F,C,G,O,T,startFound,robotMove,epsilon);    
    
    if ~flag %if new configurations were added to the search space
        iteration = iteration+1; %updating the iteration count
    end

    % Finding start config for the first time
    if(~startFound && F.inFunnel(startPose))
        
        flag = planner.addStartNodeToFunnelRRG(F,C,G,O,T,startPose);
        
        if ~flag
            iteration = iteration+1; %updating the iteration count if start config was found
            startFound=1;
            fprintf('\n\nInitial funnel-path found after <strong>%d iterations</strong>!\n\n',C.startNode.index);
        end
    end

    if mod((iteration*100/prePlanningIterationLimit),10) == 0
        waitbar(iteration/prePlanningIterationLimit)
        %fprintf('\nGenerated %0.2f percent of the funnel RRG!',iteration*100/prePlanningIterationLimit);
    end
end 
close(progressBar);

toc

if ~startFound
    error(['Couldnot compute an initial funnel-path.. Exiting in pre-planning phase itself! ' ...
        'Increase the number of samples in the next run!']);
end

%-----------------------------------------------------------%
% end of pre-planning phase
%% ----------------------------------------------------------%

% Plotting funnel tree and saving relevant data structures
if saveFlag
    fileCount = planner.saveData(F,C,O,dir,fileCount);
end

if drawFlag
    C.drawSearchGraph(); O.drawAllObstacles();   
    drawnow
    title('Constructed funnel roadmap and the computed Shortest path')
    set(gca,'FontName','Helvetica','FontSize',10, 'FontWeight','bold');
end

Q = heap(totalIterationLimit); %initialise the priority queue with the total iteration limit
C.previousRobotNode = C.startNode; C.currentRobotNode = C.startNode;

G.startVertex = G.graphVertices(C.startNode.inletVertices(1));

%Graph-search initialisation
%all nodes have infinite g and lmc value by default (constructor definition)
G.initialiseGraphSearch(Q);

%determine the best inlet to take at the start configuration
robotMoveStatus = C.findBestInletAtStartNode(G,F,Q);

fprintf('\n\n -- Expected traversal distance to goal region is <strong>%0.2f</strong> -- \n\n',G.startVertex.cost);

if ~isinf(G.startVertex.cost)
    G.drawPathToGoal();
end

%drawing the shortest path tree of search trajectories with inlets and outlets
if drawFlag
    planner.setupPlot()
    
    C.findParentInletsAtEachNode(G);
    F.constructShortestFunnelPath(G);

    F.drawSearchTrajectories(); F.drawGoalBranch();
    
    drawnow
    title('Funnel-tree and shortest Funnel-path to goal')
    set(gca,'FontName','Helvetica','FontSize',10, 'FontWeight','bold');
end

if drawFlag
    planner.setupPlot()
    F.drawGoalBranch();
    O.drawAllObstacles();
    title('Robot motion along the solution funnel-path')
    set(gca,'FontName','Helvetica','FontSize',10, 'FontWeight','bold');
    drawnow
end
%-----------------------------------------------------------%
%% start of online re-planning phase
%-----------------------------------------------------------%

%PiP-X algorithm: Online motion planning/replanning using Funnels
while (robotMoveStatus  && iteration<totalIterationLimit) || C.startNode.index ~= C.goalNode.index

    %sense obstacles
    while mod(iteration,sensingFrequency) == 0
        planner.makeDynamicChangesToGraph(F,C,G,Q,O,T);
        break
    end

    %move the robot
    while mod(iteration,robotMovementFrequency) == 0
        
        %drawing solution funnel-paths if they exist
        if drawFlag && robotMoveStatus
            O.drawAllObstacles();
            F.drawGoalBranch(); %C.drawPathToGoal();
            %plot(C.currentRobotNode.pose(1),C.currentRobotNode.pose(2), ...
            % 'dm', 'MarkerSize', 6, 'LineWidth', 3.5);
            drawnow
        end
        
        %robot-motion
        movementSkip = 1; %simulate higher robot-speed by increasing movementSkip parameter
        for movement = 1:movementSkip
            robotMoveStatus = planner.moveRobot(F,C,G,Q);
        end
        
        %print some status message and update progress variables
        if ~robotMoveStatus
        %if(isempty(C.startNode) || isinf(C.startNode.cost))
            C.startNode = C.previousRobotNode; F.startNode = C.startNode;
            idleTime = idleTime + 1; robotMove = 0;
            fprintf(['\nNo path exists currently -- Staying at the same position! ' ...
                     '\nWaiting for sampling new configurations!']);
            fprintf('\nRobot idle for %d time-steps\n',idleTime);
        else
            traversedPathLength = traversedPathLength + (C.previousRobotNode.cost - C.startNode.cost);
            remainingPathLength = C.startNode.cost;
            idleTime = 0; robotMove = 1;
            fprintf('\n\nRobot moving.... ');
            fprintf('\nTraversed distance/Remaining distance to goal - <strong>%0.2f/%0.2f</strong>',traversedPathLength,remainingPathLength);
        end
        
        break
    end
    
    %planning/replanning
    while mod(iteration,planningFrequency) == 0
        flag = planner.generateFunnelRRG(F,C,G,O,T,startFound,robotMove,epsilon);    
    
        if ~flag   %break out of this re-planning loop if and only if 
            break  %new configurations were added to the search space
        end
    end
    
    iteration = iteration+1; %updating the iteration count

    %if goal reached
    if C.goalCheck(C.startNode.pose)
        fprintf('<strong>\n\nGoal reached! \n</strong>');
        plot(C.goalNode.pose(1),C.goalNode.pose(2),'dm', 'MarkerSize', 6, 'LineWidth', 3.5);
        drawnow
        break
    end   
    
    if idleTime >= idleTimeLimit
        fprintf('\n\nCouldnot find a solution path within the allocated wait time. \nExiting!!!');
        break
    end
    
    if (saveFlag && robotMoveStatus)
        fileCount = planner.saveData(F,C,O,dir,fileCount);
    end
    
    % if(toc>120) %potentially no path exists (5 minutes of planning time)
    %     fprintf('\nNo path exists.. Exiting!')
    %     break
    % end
end
%-----------------------------------------------------------%
% end of online re-planning and robot motion
%-----------------------------------------------------------%

%% post-processing
clearvars -except planner F C G T O Q traversedPathLength drawFlag saveFlag fileCount startPose goalPose dir success

if C.goalCheck(C.startNode.pose)
    fprintf('\n\n<strong>Success!! The robot has reached the goal location!</strong>\n');
    success = 1;
else
    fprintf('\n\n<strong>Algorithm Failure!!</strong>\n');
    success = 0;

    %plotting to show robot progress
    planner.setupPlot()
    C.findParentInletsAtEachNode(G);
    F.constructShortestFunnelPath(G);
    F.drawSearchTrajectories();
    plot(C.goalNode.pose(1), C.goalNode.pose(2), 'xr', 'MarkerSize', 8, 'LineWidth', 3.5)
    plot(C.startNode.pose(1), C.startNode.pose(2), 'sg', 'MarkerSize', 8, 'LineWidth', 3.5)
    plot(C.currentRobotNode.pose(1),C.currentRobotNode.pose(2), ...
             'dm', 'MarkerSize', 6, 'LineWidth', 3.5);
    drawnow
end
 
if drawFlag
    planner.setupPlot()
    plot(C.currentRobotNode.pose(1),C.currentRobotNode.pose(2), ...
             'dm', 'MarkerSize', 6, 'LineWidth', 3.5);
    C.drawSearchGraph();
    title('Overall funnel roadmap')
    set(gca,'FontName','Helvetica','FontSize',10, 'FontWeight','bold');
    O.drawAllObstacles();
end

if saveFlag
    %2 additional frames for more 'aesthetic' video
    fileCount = planner.saveData(F,C,M,0,dir,fileCount);
    fileCount = planner.saveData(F,C,M,0,dir,fileCount);
    save([dir 'problem.mat'],'startPose','goalPose','traversedPathLength','success','fileCount');
end

toc
%-------------------------------------------------------------------------%
%end of main code