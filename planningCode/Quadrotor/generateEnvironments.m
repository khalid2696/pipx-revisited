clc; clearvars; close all

% options: 'forest', 'maze'
numTreeObstacleArray = NaN;
expType = 'maze';
numTrials = 100;

%% Folder and file management
fprintf("\n Running experiments in '<strong>%s</strong>' environment (%d trials each)\n\n", ...
        expType, numTrials);

parentDir = fullfile('.', 'presaved_environments', expType);
if ~exist(parentDir, 'dir') 
    mkdir(parentDir); 
end

for i=1:numel(numTreeObstacleArray)
    numTreeObstacles = numTreeObstacleArray(i);
    expNumber = 1;
    while expNumber <= numTrials

        fileSaveDir = fullfile(parentDir, sprintf('environment_%d', expNumber)); %sprintf('obstacles_%d', numTreeObstacles),
        if ~exist(fileSaveDir, 'dir') 
            mkdir(fileSaveDir); 
        end

        clearvars -except expType i numTreeObstacles numTreeObstacleArray numTrials expNumber ...
                            parentDir fileSaveDir
        
        try
            switch expType
                case 'forest',   run('./PiPXForestSense.m');
                case 'maze',     run('./PiPXMazeSense.m');
                otherwise, error('Unsupported experiment type');
            end
        catch ME
            if strcmpi(errorType, 'start or goal inside obstacle')
                % Environment generation issue, not the planner's fault: re-roll
                fprintf("\nStart/Goal occluded -- not algorithm's fault. Re-running the trial..\n");
                continue
            end
        end    
        
        resultFile = fullfile(fileSaveDir, 'planningProblem.mat');
        save(resultFile, 'W', 'planner');
    
        expNumber = expNumber + 1;
    end
end