clc; clearvars; close all

%% Experiment settings
% options: 'rail_dynamic', 'rail_sense'
expType = 'rail_sense';
numObstaclesArray = 3:1:6;

numTrials = 100;
gitCommitTag = '';               % for reproducibility / version control

%% Folder and file management
fprintf("\n Running experiments in '<strong>%s</strong>' environment (%d trials each)\n\n", ...
        expType, numTrials);

parentDir = fullfile('.', 'experiment_data', expType);
if ~exist(parentDir, 'dir') 
    mkdir(parentDir); 
end

% Save metadata once
metadataFile = fullfile(parentDir, 'experimentMetadata.mat');
if ~isfile(metadataFile)
    save(metadataFile, 'expType', 'numObstaclesArray', 'numTrials', 'gitCommitTag');
end

% Plain-text log to check progress remotely using terminal: 'tail -f experiment.log'
logFile = fullfile(parentDir, 'experiment.log');

%% Main loops
for expSetting1 = 1:numel(numObstaclesArray)
    numObstacles = numObstaclesArray(expSetting1);    

    expNumber = 1;
    while expNumber <= numTrials

        fileSaveDir = fullfile(parentDir, ...
            sprintf('obstacles_%g', numObstacles), ...
            sprintf('trial_%d', expNumber));
        resultFile = fullfile(fileSaveDir, 'trialResult.mat');

        % ---------- AUTOMATIC RESUME ----------
        % If a valid result already exists, skip this trial entirely.
        if isfile(resultFile)
            try
                load(resultFile, 'expOutput'); % verify readable
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

        fprintf("\nObstacles: %g, Trial id: %d", ...
                numObstacles, expNumber);
        logMessage(logFile, sprintf('START obs=%g trial=%d', ...
                numObstacles, expNumber));

        clearvars -except expType numObstaclesArray numTrials ...
            expSetting1 numObstacles expNumber ...
             parentDir fileSaveDir resultFile logFile

        errorType = ''; % ensure it always exists for the catch block
        try
            switch expType
                case 'rail_dynamic', run('./PiPXRailDynamic.m');
                case 'rail_sense',   run('./PiPXRailSense.m');
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
                logMessage(logFile, sprintf('ERROR obs=%g trial=%d :: %s', ...
                        numObstacles, expNumber, ME.message));
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

        % ---------- ATOMIC PER-TRIAL SAVE ----------
        % Write to a temp file, then rename. A crash mid-save can never
        % corrupt an existing result, and other trials are unaffected.
        saveAtomic(resultFile, expOutput);
        logMessage(logFile, sprintf('DONE  obs=%g trial=%d success=%d', ...
                numObstacles, expNumber, success));

        expNumber = expNumber + 1;

    end % trials loop
end % numObstacles loop

fprintf("\n\n Completed all %d trials of %d settings in '<strong>%s</strong>' environment\n", ...
        numTrials, numel(numObstaclesArray), expType);
fprintf("\nBuilding the master experimentsOutputTable.\n");

%% Aggregates all per-trial results into the master experimentsOutputTable
% Run this once after (or even during) the experiments. Safe to re-run anytime.

clearvars -except parentDir

load(fullfile(parentDir, 'experimentMetadata.mat'), ...
     'numObstaclesArray', 'numTrials');

experimentsOutputTable = cell(numel(numObstaclesArray), numTrials);
numMissing = 0;

for i = 1:numel(numObstaclesArray)
    for j = 1:numTrials
        resultFile = fullfile(parentDir, ...
            sprintf('obstacles_%g', numObstaclesArray(i)), ...
            sprintf('trial_%d', j), 'trialResult.mat');
        if isfile(resultFile)
            S = load(resultFile, 'expOutput');
            experimentsOutputTable{i,j} = S.expOutput;
        else
            numMissing = numMissing + 1;
        end
    end
end

% Written once, at the end
save(fullfile(parentDir, 'experimentData.mat'), 'experimentsOutputTable');

fprintf('Aggregated table saved. %d trial(s) missing/incomplete.\n', numMissing);

%% ---- Local helper functions
function saveAtomic(targetFile, expOutput)
    tmpFile = [targetFile, '.tmp'];
    save(tmpFile, 'expOutput');         
    movefile(tmpFile, targetFile);       % rename file
end

function logMessage(logFile, msg)
    fid = fopen(logFile, 'a');
    if fid ~= -1
        fprintf(fid, '[%s] %s\n', char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')), msg);
        fclose(fid);
    end
end