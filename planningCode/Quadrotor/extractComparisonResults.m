% clc; close all
clearvars
% return

%% Experiment settings
% options: 'forest_sense', 'maze_sense' 
expType = 'maze_sense';

parentDir = fullfile('.', 'experiment_data/baseline_comparison/RRTX', expType);

% Load the metadata and the experiment data
% load(fullfile(parentDir, 'experimentMetadata.mat'));

% Extract experiment output data
disp('Loading experiment output data file..');
load(fullfile(parentDir, 'experimentData.mat'), 'experimentsOutputTable', 'MCRolloutsOutputTable');

% Load the obstacle densities (~ ratio of vol(O)/vol(W)) file (for plotting)
% load('./utils/obstacleDensities_table.mat') 
% Double-verify the correspondence: numObstacles <--> density
% load('./utils/obstacleDensities_randomForest.mat')

successMetricTable = NaN(size(experimentsOutputTable)); %not collided or frozen
pathLengthMetricTable = NaN(size(experimentsOutputTable));
timeFlownMetricTable = NaN(size(experimentsOutputTable));

% experimentsOutputTable stores all the experimental data
for expSetting = 1:size(experimentsOutputTable,1) %numTreeObstacles
     % numTreeObstacles = numTreeObstaclesArray(expSetting);
        
    for expNumber = 1:size(experimentsOutputTable,2) %numTrials
       
        tempOutput = experimentsOutputTable{expSetting, expNumber};
        rolloutOutput = MCRolloutsOutputTable{expSetting, expNumber};

        if tempOutput.success == 0 %if not able to reach goal 
            successMetricTable(expSetting, expNumber) = 0;
        else
            successMetricTable(expSetting, expNumber) = mean(1-rolloutOutput.collided); %mean of no collisions
        end

        
        if isempty(rolloutOutput) %when exited in the funnel RRG construction phase itself (before moving)
            pathLengthMetricTable(expSetting, expNumber) = 0;
            timeFlownMetricTable(expSetting, expNumber) = 0;
        else
            pathLengthMetricTable(expSetting, expNumber) = mean(rolloutOutput.distFlown, 'omitnan');
            timeFlownMetricTable(expSetting, expNumber) = mean(rolloutOutput.timeFlown, 'omitnan');
        end
    end %trials loop
end %numObstacles loop

%% Print and plot statistical measures
switch expType
    case 'forest_sense'
        mean(successMetricTable,2)
        
        figure; hold on
        plot(mean(pathLengthMetricTable,2,"omitnan"));
        plot(median(pathLengthMetricTable,2,"omitnan"));
        legend('Mean of medians', 'Median of medians');
        
        figure; hold on
        plot(mean(timeFlownMetricTable,2,"omitnan"));
        plot(median(timeFlownMetricTable,2,"omitnan"));
        legend('Mean of medians', 'Median of medians');
    case 'maze_sense'
        mean(successMetricTable)
        mean(pathLengthMetricTable, "omitnan")
        median(pathLengthMetricTable, "omitnan")
        mean(timeFlownMetricTable, "omitnan")
        median(timeFlownMetricTable, "omitnan")
    otherwise
        error('Unsupported experiment type');
end
%% Saving printed metrics for later reference and paper writing

%% Maze environment with 50 trials..

%% Forest environment with 50 trials..
