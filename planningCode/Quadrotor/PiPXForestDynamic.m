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

% clc; clearvars; close all
%keyboard

%adding paths to code libraries
addpath('./lib/');

%configurable flags
drawFlag = 0;
saveFlag = 0;
videoFlag = 0;
fileCount = 1; %for saving files in /temp/ folder

%Assigning values to algorithm parameters
epsilon = 4;          %extend-distance
prePlanningIterationLimit = 150; %300 and 350
totalIterationLimit = 450; %Maximum number of iterations %keep it less than 300 always!
idleTimeLimit = 5;

planningFrequency = 1;
robotMovementFrequency = 3; %decreasing this parameter increases the robot speed!
sensingFrequency = robotMovementFrequency; %for this particular forest-sense planning problem

if ~exist('numTreeObstacles', 'var') 
    numTreeObstacles = 10; %15
end

if ~exist('obstacleDynamicity', 'var')
    obstacleDynamicity = 25; % D percent (at each sensing cycle, D*numTreeObstacles/100 obstacles would change location & size)
end

if ~exist('fileSaveDir', 'var')
    fileSaveDir = './temp/forest_dynamic/';
    mkdir(fileSaveDir);
end

envLB = 0;
envUB = 50;
obstacleSizeRange = [1 3]; %radius of circular obstacles
robotSensorRadius = 3*epsilon; %assuming robot can sense obstacles in 3 times the max move distance

W = forestEnvironment(envLB,envUB,robotSensorRadius,obstacleSizeRange,epsilon,'dynamic',obstacleDynamicity); 
%obstacle class:  epsilon - tolerance
%mode: 'sensing' or 'dynamic' (addition and deletion)

distanceFunction = @(inputA, inputB) sqrt(sum((inputA - inputB).^2,2)); %distance function (for kDTree)
T = KDTree(2, distanceFunction); %initialise the tree, 2 - num of dimensions of configuration space

load('./precomputedFunnelLibrary/library.mat');
%resolution of the pre-computed funnel library
funnelLibraryResolution = 1; %higher this resolution, finer the motion plan
F = searchFunnel(funnelLibrary,epsilon,funnelLibraryResolution);

C = configurationSpace();  %instantiate an empty configuration space class
G = searchGraph(); %augmented graph data structure to store F and C

planner = PiPxPlanner(envLB,envUB,epsilon,funnelLibraryResolution,drawFlag);
if drawFlag
    planner.setupPlot()
end

%------------------------------------%
%user-input start and goal locations
%------------------------------------%
% waitfor(msgbox('Click on the start and goal positions respectively'));
% [problemx,problemy] = ginput(2);
% 
% startPose = [problemx(1) problemy(1)];
% goalPose = [problemx(2) problemy(2)];

%--------------------------------%
%random start and goal locations
%--------------------------------%
%startPose = rand([1 2])*(W.envUB-envLB) + W.envLB;
%goalPose = rand([1 2])*(W.envUB-envLB) + W.envLB;

%--------------------------------%
%random start and goal locations around a circle of fixed distance (for experiments)
%--------------------------------%
workspaceCenter = (W.envLB + W.envUB)/2;
fixedDistance = 40;
randTheta = rand()*pi;

startPose = [workspaceCenter + fixedDistance/2*cos(randTheta), workspaceCenter + fixedDistance/2*sin(randTheta)];  
goalPose  = [workspaceCenter - fixedDistance/2*cos(randTheta), workspaceCenter - fixedDistance/2*sin(randTheta)];  

%------------------------------------------------%
%fixed start and goal locations (for dev purposes)
%------------------------------------------------%
%startPose = [5.4,4.7];
%goalPose = [45.6,44.8];

%round off to nearest integer (resolution of the motion planner)
startPose = round(startPose * funnelLibraryResolution) / funnelLibraryResolution;
goalPose = round(goalPose * funnelLibraryResolution) / funnelLibraryResolution;


%initially adding obstacles
W.addDynamicObstacles(numTreeObstacles,startPose,goalPose); %argin - #obstacles, robot pose, goal pose, 
                                                      
%W.initialiseObstacleTree();
%W.senseObstacles(startPose);

if(~W.vertexCollisionFree(goalPose) || ~W.vertexCollisionFree(startPose))
    errorType = 'Start or Goal inside obstacle';
    error('Goal inside the obstacles. No path exists!')
else
    errorType = 'None';
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
%forestEnvironment(obstacles)

%save the initial environment
if saveFlag
    fileCount = planner.saveData(F,C,G,W,fileSaveDir,fileCount);
end

%draw the initial environment
if drawFlag
    %Plotting start and goal positions
    plot(goalPose(1), goalPose(2), 'xr', 'MarkerSize', 8, 'LineWidth', 3.5)
    plot(startPose(1), startPose(2), 'sg', 'MarkerSize', 8, 'LineWidth', 3.5)
    W.drawAllObstacles(); W.drawSensorRadius(C.startNode.pose);
    drawnow
end

%keyboard
%% -----------------------------------------------------------%
% Pre-planning phase of generating a roadmap of funnels
%-----------------------------------------------------------%
if drawFlag
    progressBar = waitbar(0, 'Funnel RRG construction progress');
end

tic
while iteration < prePlanningIterationLimit %&& ~startFound
    
    flag = planner.generateFunnelRRG(F,C,G,W,T,startFound,robotMove,epsilon);    
    
    if ~flag %if new configurations were added to the search space
        iteration = iteration+1; %updating the iteration count
    end

    % Finding start config for the first time
    if(~startFound && F.inAnyInlets(startPose))
        
        flag = planner.addStartNodeToFunnelRRG(F,C,G,W,T,startPose);
        
        if ~flag
            iteration = iteration+1; %updating the iteration count if start config was found
            startFound=1; startFoundIterationCount = C.startNode.index;
            fprintf('\n\nInitial funnel-path found after <strong>%d iterations</strong>!\n\n',C.startNode.index);
        end
    end

    if mod((iteration*100/prePlanningIterationLimit),10) == 0
        if drawFlag
            waitbar(iteration/prePlanningIterationLimit)
        % else
        %     fprintf('\nGenerated %0.2f percent of the funnel RRG!',iteration*100/prePlanningIterationLimit);
        end
    end
end
preplanningComputeTime = toc; %this is the time taken to offline build the funnel RRG

if exist('progressBar', 'var') 
    close(progressBar);
end

if ~startFound
    if drawFlag
        C.drawSearchGraph();
    end
    errorType = 'exit in pre-planning phase';
    error(['Couldnot compute an initial funnel-path.. Exiting in pre-planning phase itself! ' ...
        'Increase the number of samples in the next run!']);
end

%-----------------------------------------------------------%
% end of pre-planning phase
%% ----------------------------------------------------------%

% Plotting funnel tree and saving relevant data structures
if saveFlag
    fileCount = planner.saveData(F,C,G,W,fileSaveDir,fileCount);
end

if drawFlag
    C.drawSearchGraph(); W.drawAllObstacles();   
    drawnow
    title('Constructed funnel roadmap and the computed Shortest path')
    set(gca,'FontName','Helvetica','FontSize',10, 'FontWeight','bold');
end

tic
Q = heap(totalIterationLimit); %initialise the priority queue with the total iteration limit
C.previousRobotNode = C.startNode; C.currentRobotNode = C.startNode;

G.startVertex = G.graphVertices(C.startNode.inletVertices(1));

%Graph-search initialisation
%all nodes have infinite g and lmc value by default (constructor definition)
G.initialiseGraphSearch(Q);

%determine the best inlet to take at the start configuration
disp(' ');
robotMoveStatus = C.findBestInletAtStartNode(G,F,Q);

preplanningComputeTime = preplanningComputeTime + toc; %this is the time taken to compute the initial solution funnel-path

fprintf('\n\n -- Expected traversal distance to goal region is <strong>%0.2f</strong> -- \n\n',G.startVertex.cost);

if ~isinf(G.startVertex.cost) && drawFlag
    G.drawPathToGoal();
end

%drawing the shortest path tree of search trajectories with inlets and outlets
if drawFlag
    planner.setupPlot()
    
    C.findParentInletsAtEachNode(G);
    F.constructShortestFunnelPath(G);

    F.drawSearchTrajectories(); F.drawGoalBranch();
    %F.drawSearchFunnel();
    
    drawnow
    title('Funnel-tree and shortest Funnel-path to goal')
    set(gca,'FontName','Helvetica','FontSize',10, 'FontWeight','bold');
end

%return

%-----------------------------------------------------------%
%% start of robot motion and online re-planning phase
%-----------------------------------------------------------%

if videoFlag
    writerObj = VideoWriter('sample_run.avi');
    writerObj.FrameRate = 1; % Sets the frame rate to 30 frames per second
    writerObj.Quality = 100;   % Sets the video quality (0-100)
    open(writerObj);
end

if drawFlag
    planner.setupPlot()
    F.drawGoalBranch(); W.drawAllObstacles();
    title('Robot motion along the solution funnel-path')
    set(gca,'FontName','Helvetica','FontSize',10, 'FontWeight','bold');
    drawnow

    if videoFlag
        %Capture the current figure as a frame and writes it video file
        frame = getframe(gcf);
        writeVideo(writerObj, frame);
    end
end

%PiP-X algorithm: Online motion planning/replanning using Funnels
replanningComputeTimes = NaN(totalIterationLimit - prePlanningIterationLimit,1);
traversedFunnelPath = cell(totalIterationLimit - prePlanningIterationLimit,1);
count = 1;
while (robotMoveStatus  && iteration<totalIterationLimit) || C.startNode.index ~= C.goalNode.index

    %sense obstacles
    if mod(iteration,sensingFrequency) == 0
        planner.makeDynamicChangesToGraph(F,C,G,Q,W,T);
        
        if drawFlag
            W.drawAllObstacles(); W.drawSensorRadius(C.startNode.pose);
            drawnow
            
            if videoFlag
                %Capture the current figure as a frame and writes it video file
                frame = getframe(gcf);
                writeVideo(writerObj, frame);
            end
        end
    end
    
    %planning/replanning
    if mod(iteration,planningFrequency) == 0
        
        while true %run replanning loop till we add a new config and funnel-edges
            flag = planner.generateFunnelRRG(F,C,G,W,T,startFound,robotMove,epsilon);    
            
            if flag == 0   %break out of this re-planning loop if and only if 
                break  %new configurations were added to the search space
            end        %flag = True (1) if no new configs were added, False (0) if new configs were added
        end
        
        %break
    end

    %move the robot
    if mod(iteration,robotMovementFrequency) == 0
        
        %robot-motion
        disp(' '); disp(' ');
        movementSkip = 2; %simulate higher robot-speed by increasing movementSkip parameter
        for movement = 1:movementSkip

            %This is the step that computes solution funnel-path,
            %so analyse computation times for replanning
            tic
            [robotMoveStatus, traversedFunnelEdge] = planner.moveRobot(F,C,G,Q);
            replanningComputeTimes(count) = toc; 
            if robotMoveStatus
                traversedFunnelPath{count} = traversedFunnelEdge;
            end
            count = count+1;

            %if goal reached
            if C.goalCheck(C.startNode.pose)
                traversedPathLength = traversedPathLength + (C.previousRobotNode.cost - C.startNode.cost);
                remainingPathLength = C.startNode.cost;
                fprintf('\n\nTraversed distance/Remaining distance to goal - <strong>%0.2f/%0.2f</strong>', ...
                    traversedPathLength,remainingPathLength);
                fprintf('<strong>\n\nGoal reached! \n</strong>');
                if drawFlag
                    plot(C.goalNode.pose(1),C.goalNode.pose(2),'dm', 'MarkerSize', 6, 'LineWidth', 3.5);
                    W.drawAllObstacles();
                    drawnow
                end

                break
            end

            if robotMoveStatus
                traversedPathLength = traversedPathLength + (C.previousRobotNode.cost - C.startNode.cost);
                remainingPathLength = C.startNode.cost;
                idleTime = 0; robotMove = 1;
                fprintf('\n\nRobot moving.... ');
                fprintf('\nTraversed distance/Remaining distance to goal - <strong>%0.2f/%0.2f</strong>', ...
                    traversedPathLength,remainingPathLength);
            end
        end

        %print some status message and update progress variables
        if ~robotMoveStatus
            C.startNode = C.previousRobotNode; F.startNode = C.startNode;
            idleTime = idleTime + 1; robotMove = 0;
            fprintf(['\nNo path exists currently -- Staying at the same position! ' ...
                     '\nWaiting for sampling new configurations!']);
            fprintf('\nRobot idle for %d time-steps\n',idleTime);
        end

    end

    %plotting replanned funnel-path as robot moves
    if drawFlag
         if mod(iteration,robotMovementFrequency) == 0 && robotMoveStatus %drawing solution funnel-paths if they exist
            %planner.setupPlot(); C.drawSearchTree();  
            W.drawAllObstacles(); %W.drawSensorRadius(C.startNode.pose);
            F.drawGoalBranch(); %C.drawPathToGoal();
            %plot(C.currentRobotNode.pose(1),C.currentRobotNode.pose(2), ...
            % 'dm', 'MarkerSize', 6, 'LineWidth', 3.5);
            drawnow
            
            if videoFlag
                %Capture the current figure as a frame and writes it video file
                frame = getframe(gcf);
                writeVideo(writerObj, frame);
            end
        end
    end
    
    iteration = iteration+1; %updating the iteration count

    %if goal reached
    if C.goalCheck(C.startNode.pose)
        fprintf('<strong>\n\nGoal reached! \n</strong>');
        if drawFlag
            plot(C.goalNode.pose(1),C.goalNode.pose(2),'dm', 'MarkerSize', 6, 'LineWidth', 3.5);
            W.drawAllObstacles();
            drawnow
        end
        break
    end   
    
    if idleTime >= idleTimeLimit
        fprintf('\n\nCouldnot find a solution path within the allocated wait time. \nExiting!!!');
        break
    end
    
    if (saveFlag && robotMoveStatus)
        fileCount = planner.saveData(F,C,G,W,fileSaveDir,fileCount);
    end
    
    % if(toc>120) %potentially no path exists (5 minutes of planning time)
    %     fprintf('\nNo path exists.. Exiting!')
    %     break
    % end
end

if videoFlag
    close(writerObj);
    disp('Video created successfully!');
end

replanningComputeTimes = replanningComputeTimes(1:count-1);
traversedFunnelPath = traversedFunnelPath(1:count-1);

% replanningComputeTimeQuantiles = quantile(replanningComputeTimes, [0.1, 0.5, 0.9]);
% replanningFrequencyQuantiles = 1./replanningComputeTimeQuantiles;

%-----------------------------------------------------------%
% end of online re-planning and robot motion
%-----------------------------------------------------------%

%% post-processing
%clearvars -except planner F C G T W Q traversedPathLength drawFlag saveFlag ...
%                    fileCount startPose goalPose dir success iteration

if C.goalCheck(C.startNode.pose)
    fprintf('\n\n<strong>Success!! The robot has reached the goal location!</strong>\n');
    success = 1;
else
    fprintf('\n\n<strong>Algorithm Failure!!</strong>\n');
    success = 0;

    %plotting to show robot progress
    if drawFlag
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
end
 
if drawFlag
    planner.setupPlot()
    plot(C.currentRobotNode.pose(1),C.currentRobotNode.pose(2), ...
             'dm', 'MarkerSize', 6, 'LineWidth', 3.5);
    C.drawSearchGraph();
    title('Overall funnel roadmap')
    set(gca,'FontName','Helvetica','FontSize',10, 'FontWeight','bold');
    W.drawAllObstacles();
end

if saveFlag
    %2 additional frames for more 'aesthetic' video
    fileCount = planner.saveData(F,C,G,W,fileSaveDir,fileCount);
    fileCount = planner.saveData(F,C,G,W,fileSaveDir,fileCount);
    save([fileSaveDir 'problem.mat'],'startPose','goalPose','traversedPathLength','success','fileCount');
end

%-------------------------------------------------------------------------%
%end of main code