clc; clearvars; close all
% return

%adding paths to code libraries
addpath('./lib/');

%% Experiment settings
% options: 'forest_sense', 'maze_sense'
usePresavedEnvironments = true; 
expType = 'forest_sense';
numTreeObstaclesArray = 5:5:75; %5:5:75
numTrials = 10;
gitCommitTag = '04c9e3a';               % for reproducibility / version control

%% Folder and file management
fprintf("\n Running experiments in '<strong>%s</strong>' environment (%d trials each)\n\n", ...
        expType, numTrials);

% parentDir = fullfile('.', 'experiment_data/baseline_comparison/PiPX', expType);
parentDir = fullfile('.', 'temp_invariance_verification', expType);
if ~exist(parentDir, 'dir') 
    mkdir(parentDir); 
end

% Save metadata once
metadataFile = fullfile(parentDir, 'experimentMetadata.mat');
if ~isfile(metadataFile)
    save(metadataFile, 'expType', 'numTreeObstaclesArray', 'numTrials', 'gitCommitTag');
end

% Plain-text log to check progress remotely using terminal: 'tail -f experiment.log'
logFile = fullfile(parentDir, 'experiment.log');

for expSetting1 = 1:numel(numTreeObstaclesArray)
    numTreeObstacles = numTreeObstaclesArray(expSetting1);

    expNumber = 1;
    while expNumber <= numTrials
    
        fileSaveDir = fullfile(parentDir, ...
            sprintf('obstacles_%g', numTreeObstacles), sprintf('trial_%d', expNumber));
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
    
        fprintf("\nObstacles: %g, Trial id: %d", numTreeObstacles, expNumber);
        logMessage(logFile, sprintf('START obs=%g trial=%d', numTreeObstacles, expNumber));
    
        clearvars -except usePresavedEnvironments expType expSetting1 numTreeObstaclesArray ...
                            numTreeObstacles numTrials expNumber parentDir fileSaveDir resultFile logFile
    
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
end %numObstacles loop
   
fprintf("\n\n Completed all %d trials in '<strong>%s</strong>' environment\n", ...
        numTrials, expType);

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