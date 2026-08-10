clc; clearvars; close all
% return

%% Experiment settings
% options: 'forest_sense', 'maze_sense'
expType = 'maze_sense';
numTreeObstacles = 5;
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

    clearvars -except expType numTreeObstacles numTrials expNumber parentDir fileSaveDir resultFile logFile

    errorType = ''; % ensure it always exists for the catch block
    try
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

clearvars -except parentDir

load(fullfile(parentDir, 'experimentMetadata.mat'), 'numTrials');

numMissing = 0;

for k = 1:numTrials
    resultFile = fullfile(parentDir, sprintf('trial_%d', k), 'trialResult.mat');
    if isfile(resultFile)
        load(resultFile, 'expOutput', 'expDataStructures');
        postProcess(expOutput, expDataStructures);
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

for k = 1:numTrials
    resultFile = fullfile(parentDir, sprintf('trial_%d', k), 'trialResult.mat');
    if isfile(resultFile)
        S = load(resultFile, 'expOutput');
        experimentsOutputTable{k} = S.expOutput;
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

function postProcess(expOutput, expDataStructures)
    F = expDataStructures.funnelRoadmap; G = expDataStructures.searchGraph;
    C = expDataStructures.configSpace; W = expDataStructures.workSpace;
    planner = expDataStructures.planner;

    planner.setupPlot(); W.drawAllObstacles();
    traversedFunnelPath = expOutput.traversedFunnelPath;

    for i=1:numel(traversedFunnelPath)
        tempFunnel = traversedFunnelPath{i}
        F.drawFunnel(tempFunnel,2);
    end

    plot(planner.goalConfig(1), planner.goalConfig(2), 'xr', 'MarkerSize', 8, 'LineWidth', 3.5)
    plot(planner.startConfig(1), planner.startConfig(2), 'sg', 'MarkerSize', 8, 'LineWidth', 3.5)
            
end