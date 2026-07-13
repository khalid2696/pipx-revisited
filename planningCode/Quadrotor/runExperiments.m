clc; clearvars; close all

%experiment settings
expType = 'forest_sense';
numTreeObstaclesArray = 0:5:10; %75
obstacleDynamicityArray = 0;
numTrials = 2;
gitCommitTag = ''; %saving this for reproducibility and version control

fprintf("\n Running experiments in '<strong>%s</strong>' environment (%d trials each)\n\n", expType, numTrials);

%create folder for saving experiment data
parentDir = ['./experiment_data/' expType];
if exist(parentDir, 'dir')
    rmdir(parentDir, 's'); %remove folder if already exists from previous runs
end
mkdir(parentDir);

% saving metadata in the output file 
experimentsDataFilePath = fullfile(parentDir, '/experimentData.mat');
save(experimentsDataFilePath, 'expType', 'numTreeObstaclesArray', 'obstacleDynamicityArray', 'numTrials', 'gitCommitTag');

experimentsOutputTable = cell(numel(numTreeObstaclesArray), numel(obstacleDynamicityArray), numTrials);

for expSetting1 = 1:numel(numTreeObstaclesArray)

    numTreeObstacles = numTreeObstaclesArray(expSetting1);

    for expSetting2 = 1:numel(obstacleDynamicityArray)
        
        obstacleDynamicity = obstacleDynamicityArray(expSetting2);

        for expNumber = 1:numTrials
            
            expNumber
    
            fileSaveDir = [parentDir '/obstacles_' num2str(numTreeObstacles) ...
                                '/trial_' num2str(expNumber) '/'];
            if ~exist('fileSaveDir','dir')
                mkdir(fileSaveDir);
            end
        
            clearvars -except expType numTreeObstaclesArray obstacleDynamicityArray numTrials ...
                                expSetting1 numTreeObstacles expSetting2 obstacleDynamicity  expNumber...
                                  parentDir fileSaveDir experimentsDataFilePath experimentsOutputTable 
            
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
            experimentsOutputTable{expSetting1,expSetting2,expNumber} = expOutput;
        
            %save the data generated from the experiments (after each trial for safety)
            save(experimentsDataFilePath, 'experimentsOutputTable', '-append');
        
        end %MC trials loop
    end %obstacleDynamicity loop
end %numObstacles loop

fprintf("\n\n Completed running all %d trials of %d settings in '<strong>%s</strong>' environment\n", ...
                numTrials, numel(numTreeObstaclesArray)*numel(obstacleDynamicityArray), expType);