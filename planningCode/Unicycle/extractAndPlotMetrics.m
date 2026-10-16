clc; clearvars; close all
% return

%select experiment setting
%Options: 'forest_sense', 'road_sense'
expType = 'forest_sense';
parentDir = fullfile('.', 'experiment_data', expType);

% Load the metadata and the experiment data 
load(fullfile(parentDir, 'experimentMetadata.mat'), 'numObstaclesArray', 'numTrials');

% Extract experiment output data
disp('Loading experiment output data file..');
load(fullfile(parentDir, 'experimentData.mat'), 'experimentsOutputTable'); 

successMetricTable = NaN(size(experimentsOutputTable));
pathLengthMetricTable = NaN(size(experimentsOutputTable));

% experimentsOutputTable stores all the experimental data
for expSetting1 = 1:size(experimentsOutputTable,1)-1 %numTreeObstacles
    % numTreeObstacles = numTreeObstaclesArray(expSetting1);
    
    for expNumber = 1:size(experimentsOutputTable,2) %numTrials
       
        tempOutput = experimentsOutputTable{expSetting1, expNumber}; 

        % tempOutput.success
        % tempOutput.funnelPathCost
        % tempOutput.traversedFunnelPath %isempty(traversedFunnelPath{index}) to figure out if the robot was idle
        % tempOutput.preplanningComputeTime
        % tempOutput.startFoundIterationCount
        % tempOutput.onlineReplanningComputeTimes
        % tempOutput.errorType

        successMetricTable(expSetting1, expNumber) = tempOutput.success;
        pathLengthMetricTable(expSetting1, expNumber) = tempOutput.funnelPathCost;

    end %MC trials loop
end %numObstacles loop

%% Extracting statistical measures (mean/std_dev or median/quantiles) and plotting

fprintf('\nSuccess rate for different numObstacles (numTrials in each setting = %d):\n', numTrials);
fprintf('\nNumber of obstacles:');
disp(numObstaclesArray(1:end-1));
fprintf('Success rate:')
successRate = mean(successMetricTable, 2, "omitnan")';
disp(successRate);
fprintf("Success rate across all numObstacles: %.2f\n", mean(successRate,"omitnan")*100);

%% printed forest_sense results (for later reference and paper writing)
% Success rate for different numObstacles (numTrials in each setting = 100):
% Number of obstacles:    15    20    25
% Success rate:    0.9800    1.0000    0.9300
% Success rate across all numObstacles: 97.00

%% printed forest_sense results (for later reference and paper writing)
% Success rate for different numObstacles (numTrials in each setting = 100):
% Number of obstacles:     5     7     9    11
% Success rate:    0.9900    0.9700    0.9400    0.8800 
% Success rate across all numObstacles: 94.50