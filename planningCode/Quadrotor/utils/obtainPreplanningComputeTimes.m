clc; clearvars; close all
% return

%% Experiment settings
expType = 'forest_sense';
numTreeObstaclesArray = 15;            % e.g. 45
preplanningIterationLimitArray = 0:50:400; preplanningIterationLimitArray(1) = 25;
numTrials = 100;
gitCommitTag = 'bce96e1';               % for reproducibility / version control

%% Folder and file management
fprintf("\n Running experiments in '<strong>%s</strong>' environment (%d trials each)\n\n", ...
        expType, numTrials);

parentDir = fullfile('.', 'experiment_data', expType);
if ~exist(parentDir, 'dir') 
    mkdir(parentDir); 
end

preplanningComputeTimeData = NaN(numel(numTreeObstaclesArray), numel(preplanningIterationLimitArray), numTrials);
%% Main loops
for expSetting1 = 1:numel(numTreeObstaclesArray)
    numTreeObstacles = numTreeObstaclesArray(expSetting1);

    for expSetting2 = 1:numel(preplanningIterationLimitArray)
        prePlanningIterationLimit = preplanningIterationLimitArray(expSetting2);

        expNumber = 1;
        while expNumber <= numTrials

            clearvars -except expType numTreeObstaclesArray preplanningIterationLimitArray numTrials ...
                expSetting1 numTreeObstacles expSetting2 prePlanningIterationLimit expNumber preplanningComputeTimeData...

            errorType = ''; % ensure it always exists for the catch block
            try
                switch expType
                    case 'forest_sense',   run('../PiPXForestSense.m');
                    case 'maze_sense',     run('../PiPXMazeSense.m');
                    otherwise, error('Unsupported experiment type');
                end
            catch ME
                if strcmpi(errorType, 'start or goal inside obstacle')
                    % Environment generation issue, not the planner's fault: re-roll
                    fprintf("\nStart/Goal occluded -- not algorithm's fault. Re-running the trial..\n");
                    continue
                elseif strcmpi(errorType, 'exit in pre-planning phase')
                    % startFoundIterationCount = NaN;
                    % preplanningComputeTime = NaN;
                    warning("\nExited in the pre-planning stage itself!\n");
                else
                    % Unexpected error: record it (with the actual message) and move on
                    error('Unexpected error in trial %d: %s', expNumber, ME.message);
                end
            end
            
            preplanningComputeTimeData(expSetting1,expSetting2,expNumber) = preplanningComputeTime;

            expNumber = expNumber + 1;

        end % trials loop
    end % preplanning iteration limit loop
    
    fprintf("\n\n Completed all %d trials of %d settings in '<strong>%s</strong>' environment with %d obstacles\n", ...
        numTrials, numel(preplanningIterationLimitArray), expType, numTreeObstacles);
end % numObstacles loop

%% Prints metrics

clearvars -except expType preplanningComputeTimeData preplanningIterationLimitArray numTrials numTreeObstaclesArray gitCommitTag
switch expType
    case 'forest_sense'
        fprintf("\nMetrics from forest environment:\n\n");   
        disp('Preplanning iteration limits');
        disp(preplanningIterationLimitArray);
        disp('Median preplanning times (in s):');
        medianPreplanningComputeTime = median(preplanningComputeTimeData, 3, "omitnan"); %along third dimension (numTrials)
        disp(medianPreplanningComputeTime); 
    case 'maze_sense'
        fprintf("\nMetrics from maze environment");   
        disp('Preplanning iteration limits');
        disp(preplanningIterationLimitArray);
        disp('Median preplanning times:');
        medianPreplanningComputeTime = median(preplanningComputeTimeData, 3, "omitnan"); %along third dimension (numTrials)
        disp(medianPreplanningComputeTime); 
    otherwise
        error('Unsupported experiment type');
end

%% Printed output (for future reference and paper writing)
% load('preplanningComputeTimeAnalysis.mat');
% Metrics from forest environment:
% Preplanning iteration limits
%     25    50   100   150   200   250   300   350   400
% Median preplanning times (in s):
%     0.1721    0.5436    2.0061    4.9789    9.3465   15.6966   25.4752   38.5861   55.2600