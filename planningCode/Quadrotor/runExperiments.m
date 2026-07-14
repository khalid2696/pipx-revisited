clc; clearvars; close all

% Experiment settings
% options: 'forest_sense', 'forest_dynamic', 'forest_moving', 'maze_sense', 'maze_dynamic'
expType = 'forest_sense'; 
numTreeObstaclesArray = 0:5:75; %75
obstacleDynamicityArray = NaN; %50 seems to be the limit 
numTrials = 25;
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

        expNumber = 1;
        while expNumber <= numTrials
            
            fprintf("\nObstacles: %d, Dynamicity: %d, Trial id: %d", ...
                        numTreeObstacles, obstacleDynamicity, expNumber);
    
            fileSaveDir = [parentDir '/obstacles_' num2str(numTreeObstacles) ...
                             '_dynamicity_' num2str(obstacleDynamicity) '/trial_' num2str(expNumber) '/'];
            if ~exist('fileSaveDir','dir')
                mkdir(fileSaveDir);
            end
        
            clearvars -except expType numTreeObstaclesArray obstacleDynamicityArray numTrials ...
                                expSetting1 numTreeObstacles expSetting2 obstacleDynamicity  expNumber...
                                  parentDir fileSaveDir experimentsDataFilePath experimentsOutputTable 
            
            try
                switch expType
                    case 'forest_sense'
                        run('./PiPXForestSense.m');
                    case 'forest_dynamic'
                        run('./PiPXForestDynamic.m');
                    case 'forest_moving'
                        run('./PiPXForestMoving.m');
                    case 'maze_sense'
                        run('./PiPXMazeSense.m');
                    case 'maze_dynamic'
                        run('./PiPXMazeDynamic.m');
                    otherwise
                        error('Unsupported experiment type')
                end
            catch
                %2 error types: 'Start or Goal inside obstacle' and 'exit in pre-planning phase'
                %Re-run the experiment in case of first error (not our planner's error)
                
                if strcmpi(errorType, 'start or goal inside obstacle')
                    expNumber = expNumber - 1;
                    fprintf("\nStart/Goal occluded -- not algorithm's fault. Re-running the trial..\n");
                    continue
                elseif strcmpi(errorType, 'exit in pre-planning phase')
                    success = 0;
                    traversedPathLength = NaN;
                    startFoundIterationCount = NaN;
                    replanningComputeTimes = [];
                    traversedFunnelPath = [];
                else
                    warning('Unknown error!! Should not happen, check the code!');
                    success = 0;
                    traversedPathLength = NaN;
                    % startFoundIterationCount = NaN;
                    % preplanningComputeTime = NaN;
                    % replanningComputeTimes = [];
                    % traversedFunnelPath = [];
                end
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
            
            %increment the experiment counter
            expNumber = expNumber + 1;
        
        end %MC trials loop
    end %obstacleDynamicity loop
end %numObstacles loop

fprintf("\n\n Completed running all %d trials of %d settings in '<strong>%s</strong>' environment\n", ...
                numTrials, numel(numTreeObstaclesArray)*numel(obstacleDynamicityArray), expType);