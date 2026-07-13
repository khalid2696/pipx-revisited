clc; clearvars; close all

expType = 'forest_sense';
parentDir = ['./experiment_data/' expType];
if exist('parentDir','dir')
    rmdir(parentDir, 's');
end
mkdir(parentDir);

numTreeObstaclesArray = [0, 5, 10];
numTrials = 2;
gitCommitTag = ''; %saving this for reproducibility and version control

% saving metadata in the output file (remove if already exists from previous runs)
experimentsDataFilePath = fullfile(parentDir, '/experimentData.mat');
% if exist(experimentsDataFilePath, 'file') == 2
%     delete(experimentsDataFilePath);
% end
save(experimentsDataFilePath, 'expType', 'numTreeObstaclesArray', 'numTrials', 'gitCommitTag');

experimentsOutputTable = cell(numel(numTreeObstaclesArray), numTrials);
for expCondition = 1:numel(numTreeObstaclesArray)

    numTreeObstacles = numTreeObstaclesArray(expCondition)

    for expNumber = 1:numTrials
        
        expNumber

        fileSaveDir = [parentDir '/obstacles_' num2str(numTreeObstacles) ...
                            '/trial_' num2str(expNumber) '/'];
        if ~exist('fileSaveDir','dir')
            mkdir(fileSaveDir);
        end
    
        clearvars -except expType numTreeObstaclesArray numTrials fileSaveDir...
                            expCondition expNumber numTreeObstacles ...
                              parentDir experimentsDataFilePath experimentsOutputTable 
        
        try
            run('./PiPXForestSense.m');
        catch
            if strcmpi(errorType, 'start or goal inside obstacle')
                expNumber = expNumber - 1;
                fprintf("\nNot algorithm's fault. Re-running the trial..\n");
                continue
            end

            %2 error types: 'Start or Goal inside obstacle' and 'exit in pre-planning phase'
            %Re-run the experiment in case of first error (not our planner's error)
            success = 0;
            traversedPathLength = NaN;
            startFoundIterationCount = NaN;
            replanningComputeTimes = [];
            traversedFunnelPath = [];
        end
    
        %successful trial implies no runtime errors
        if success == 1
            errorType = 'None';
        else %if the robot didnot reach the goal, void the datapoint on traversed path length
            traversedPathLength = NaN;
        end
    
        %save experiment outputs into a struct variable
        expOutput = struct('success', [], 'funnelPathCost', [], 'traversedFunnelPath', [], ...
                            'preplanningComputeTime', [], 'startFoundIterationCount', [], ...
                            'onlineReplanningComputeTimes', [], 'errorType', []);
             
        expOutput.success = success;
        expOutput.funnelPathCost = traversedPathLength;
        expOutput.traversedFunnelPath = traversedFunnelPath; %isempty(traversedFunnelPath{index}) to figure out if the robot was idle
        expOutput.preplanningComputeTime = preplanningComputeTime;
        expOutput.startFoundIterationCount = startFoundIterationCount;
        expOutput.onlineReplanningComputeTimes = replanningComputeTimes;
        expOutput.errorType = errorType;
        
        %save into master output table
        experimentsOutputTable{expCondition,expNumber} = expOutput;
    
        %save the data generated from the experiments (after each trial for safety)
        save(experimentsDataFilePath, 'experimentsOutputTable', '-append');
    end
end

fprintf("\n\n Completed running all %d trials in '<strong>%s</strong>' environment\n", numTrials, expType)
return
%% Post-processing

clearvars -except expType numTrials numTreeObstacles successArray lengthArray replanningComputeTimeHistory traversedFunnelPath
    
replanningComputeTimeQuantiles = quantile(replanningComputeTimeHistory, [0.1, 0.5, 0.9]);

% Printing some metadata and results
fprintf('\n\nEnvironment: %s, Number of obstacles: %d (%d trials)', expType, numTreeObstacles, numTrials);
fprintf('\nSuccess rate                  -- mean: <strong>%0.2f</strong>, std dev: <strong>%0.2f</strong>', ...
                    mean(successArray),std(successArray));
fprintf('\nTraversed trajectory length -- mean: <strong>%0.2f</strong>, std dev: <strong>%0.2f</strong>', ...
                    mean(lengthArray, 'omitnan'),std(lengthArray, 'omitnan'));
fprintf('\nReplanning compute times -- median: <strong>%0.3f</strong> s, (0.1,0.9)-quant: (<strong>%0.3f</strong>, <strong>%0.3f</strong>) s', ...
                    replanningComputeTimeQuantiles(2),replanningComputeTimeQuantiles(1),replanningComputeTimeQuantiles(3));
fprintf('\nReplanning frquencies -- median: <strong>%0.2f</strong> Hz, (0.1,0.9)-quant: (<strong>%0.2f</strong>, <strong>%0.2f</strong>) Hz', ...
                    1/replanningComputeTimeQuantiles(2),1/replanningComputeTimeQuantiles(1),1/replanningComputeTimeQuantiles(3));
fprintf('\n\n');