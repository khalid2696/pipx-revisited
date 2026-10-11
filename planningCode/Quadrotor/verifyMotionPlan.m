clc; clearvars; close all
% return

%adding paths to code libraries
addpath('./lib/');

%% Experiment settings
% options: 'forest_sense', 'maze_sense'
usePresavedEnvironments = true; 
expType = 'maze_sense';
numTreeObstacles = 10; %5:5:75
numTrials = 1;
gitCommitTag = '';               % for reproducibility / version control

%% Folder and file management
fprintf("\n Running experiments in '<strong>%s</strong>' environment (%d trials each)\n\n", ...
        expType, numTrials);

% parentDir = fullfile('.', 'experiment_data/invariance_verification', expType);
parentDir = fullfile('.', 'temp_invariance_verification', expType);
if ~exist(parentDir, 'dir') 
    mkdir(parentDir); 
end

% Save metadata once
metadataFile = fullfile(parentDir, 'experimentMetadata.mat');
if ~isfile(metadataFile)
    save(metadataFile, 'expType', 'numTrials', 'gitCommitTag');
end

% Plain-text log to check progress remotely using terminal: 'tail -f experiment.log'
logFile = fullfile(parentDir, 'experiment.log');

expNumber = 1;
while expNumber <= numTrials

    fileSaveDir = fullfile(parentDir, sprintf('trial_%d', expNumber));
    resultFile = fullfile(fileSaveDir, 'trialResult.mat');

    % ---------- AUTOMATIC RESUME ----------
    % If a valid result already exists, skip this trial entirely.
    if isfile(resultFile)
        try
            load(resultFile, 'expOutput', 'expDataStructures'); % verify readable
            expNumber = expNumber + 1;
            continue
        catch
            % Corrupt/partial file from a previous crash: redo the trial
            delete(resultFile);
        end
    end
    if ~exist(fileSaveDir, 'dir') 
        mkdir(fileSaveDir); 
    end

    fprintf("\nTrial id: %d", expNumber);
    logMessage(logFile, sprintf('START trial=%d', expNumber));

    clearvars -except usePresavedEnvironments expType numTreeObstacles numTrials ...
                        expNumber parentDir fileSaveDir resultFile logFile

    errorType = ''; % ensure it always exists for the catch block
    try
        environmentID = expNumber;
        switch expType
            case 'forest_sense',   run('./PiPXForestSense.m');
            case 'maze_sense',     run('./PiPXMazeSense.m');
            otherwise, error('Unsupported experiment type');
        end
    catch ME
        if strcmpi(errorType, 'start or goal inside obstacle')
            % Environment generation issue, not the planner's fault: re-roll
            fprintf("\nStart/Goal occluded -- not algorithm's fault. Re-running the trial..\n");
            continue
        elseif strcmpi(errorType, 'exit in pre-planning phase')
            success = 0;
            traversedPathLength = NaN;
            startFoundIterationCount = NaN;
            preplanningComputeTime = NaN;
            replanningComputeTimes = [];
            traversedFunnelPath = [];
            fprintf("\nExited in the pre-planning stage itself!\n");
        else
            % Unexpected error: record it (with the actual message) and move on
            warning('Unexpected error in trial %d: %s', expNumber, ME.message);
            logMessage(logFile, sprintf('ERROR trial=%d :: %s', expNumber, ME.message));
            success = 0;
            traversedPathLength = NaN;
            % startFoundIterationCount = NaN;
            % preplanningComputeTime = NaN;
            % replanningComputeTimes = [];
            % traversedFunnelPath = [];
            errorType = ME.message;
        end
    end

    if success == 1
        errorType = 'None';
    else
        traversedPathLength = NaN;
    end

    % Package this trial's output
    expOutput = struct();
    expOutput.success                     = success;
    expOutput.funnelPathCost              = traversedPathLength;
    expOutput.traversedFunnelPath         = traversedFunnelPath; %isempty(traversedFunnelPath{index}) to figure out if the robot was idle
    expOutput.preplanningComputeTime      = preplanningComputeTime;
    expOutput.startFoundIterationCount    = startFoundIterationCount;
    expOutput.onlineReplanningComputeTimes = replanningComputeTimes;
    expOutput.errorType                   = errorType;

    % Package this trial's data structures (to post-process)
    expDataStructures = struct();
    expDataStructures.planner = planner;
    expDataStructures.funnelRoadmap = F;
    expDataStructures.configSpace = C;
    expDataStructures.workSpace = W;
    expDataStructures.searchGraph = G;

    % ---------- ATOMIC PER-TRIAL SAVE ----------
    % Write to a temp file, then rename. A crash mid-save can never
    % corrupt an existing result, and other trials are unaffected.
    saveAtomic(resultFile, expOutput, expDataStructures);
    logMessage(logFile, sprintf('DONE  trial=%d success=%d', expNumber, success));

    expNumber = expNumber + 1;

end % trials loop
   
fprintf("\n\n Completed all %d trials in '<strong>%s</strong>' environment\n", ...
        numTrials, expType);

%% Post processing
fprintf("\nPost processing: running trajectory rollouts to verify invariance..\n");

close all
clearvars -except parentDir

%adding paths to code libraries (required for parsing data structures properly)
addpath('./lib/');

load(fullfile(parentDir, 'experimentMetadata.mat'), 'numTrials');

numMissing = 0;

for trialID = 1:numTrials

    clearvars -except parentDir trialID

    resultFile = fullfile(parentDir, sprintf('trial_%d', trialID), 'trialResult.mat');
    if isfile(resultFile)
        load(resultFile, 'expOutput', 'expDataStructures');

        % Run perturbations robustness script
        % Requires var names: 'funnelPath' and 'obstacles' (so extract from experiment output data)
        [funnelPath, obstacles] = postProcess(expOutput, expDataStructures);
        run('./checkFunnelPathInvariance_Perturbations.m');
    else
        numMissing = numMissing + 1;
    end
end

return
%% Aggregates all per-trial results into the master experimentsOutputTable
% Run this once after (or even during) the experiments. Safe to re-run anytime.

fprintf("\nBuilding the master experimentsOutputTable.\n");

clearvars -except parentDir

load(fullfile(parentDir, 'experimentMetadata.mat'), 'numTrials');

experimentsOutputTable = cell(numTrials);
numMissing = 0;

for trialID = 1:numTrials
    resultFile = fullfile(parentDir, sprintf('trial_%d', trialID), 'trialResult.mat');
    if isfile(resultFile)
        S = load(resultFile, 'expOutput');
        experimentsOutputTable{trialID} = S.expOutput;
    else
        numMissing = numMissing + 1;
    end
end

% Written once, at the end
save(fullfile(parentDir, 'experimentData.mat'), 'experimentsOutputTable');

fprintf('Aggregated table saved. %d trial(s) missing/incomplete.\n', numMissing);

%% ---- Local helper functions
function saveAtomic(targetFile, expOutput, expDataStructures)
    tmpFile = [targetFile, '.tmp'];
    save(tmpFile, 'expOutput', 'expDataStructures');         
    movefile(tmpFile, targetFile);       % rename file
end

function logMessage(logFile, msg)
    fid = fopen(logFile, 'a');
    if fid ~= -1
        fprintf(fid, '[%s] %s\n', char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')), msg);
        fclose(fid);
    end
end

function [funnelPath, obstacles] = postProcess(expOutput, expDataStructures)
    F = expDataStructures.funnelRoadmap; G = expDataStructures.searchGraph;
    C = expDataStructures.configSpace; W = expDataStructures.workSpace;
    planner = expDataStructures.planner;

    % extracting the traversed funnel path and list of obstacles
    funnelPath = expOutput.traversedFunnelPath;

    % Making a list of obstacles to collision check
    obstacles = NaN(numel(W.obstacles), 3); %3 - [xc yc r]
    for i=1:numel(W.obstacles)
        tempObstacle = W.obstacles{i};
        if tempObstacle.status == 0 %unsensed, so skip collision checking
            continue
        end
        obstacles(i,1:2) = tempObstacle.location;
        obstacles(i,3) = tempObstacle.radius;
    end
    obstacles = rmmissing(obstacles); %remove any inactive obstacles from the list

    % visualizing traversed funnel path
    planner.setupPlot(); W.drawAllObstacles();

    for i=1:numel(funnelPath)
        tempFunnel = funnelPath{i};
        if isempty(tempFunnel) %skip empty saved data (happens due to nature of file saving)
            continue
        end
        F.drawFunnel(tempFunnel,2);
    end

    plot(planner.goalConfig(1), planner.goalConfig(2), 'xr', 'MarkerSize', 8, 'LineWidth', 3.5)
    plot(planner.startConfig(1), planner.startConfig(2), 'sg', 'MarkerSize', 8, 'LineWidth', 3.5)
            
end