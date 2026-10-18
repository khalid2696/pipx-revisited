clc; close all; clearvars
% return

%% Experiment settings
% options: 'forest_sense', 'maze_sense' 
expType = 'maze_sense';

parentDir = fullfile('.', 'experiment_data/baseline_comparison/PiPX', expType);

% Load the metadata and the experiment data
% load(fullfile(parentDir, 'experimentMetadata.mat'));

% Extract experiment output data
disp('Loading experiment output data file..');
load(fullfile(parentDir, 'experimentData.mat'), 'experimentsOutputTable', 'MCRolloutsOutputTable');

% Load the obstacle densities (~ ratio of vol(O)/vol(W)) file (for plotting)
% load('./utils/obstacleDensities_table.mat') 
% Double-verify the correspondence: numObstacles <--> density
% load('./utils/obstacleDensities_randomForest.mat')

% mean
successMetricTable = NaN(size(experimentsOutputTable)); %not collided or frozen
%(0, 0.1, 0.5, 0.9, 1 quantiles)
pathLengthMetricTable = NaN(size(experimentsOutputTable,1), size(experimentsOutputTable,2), 5);
timeFlownMetricTable = NaN(size(experimentsOutputTable,1), size(experimentsOutputTable,2), 5);

% experimentsOutputTable stores all the experimental data
for expSetting = 1:size(experimentsOutputTable,1) %numTreeObstacles
        
    for expNumber = 1:size(experimentsOutputTable,2) %numTrials
       
        tempOutput = experimentsOutputTable{expSetting, expNumber};
        rolloutOutput = MCRolloutsOutputTable{expSetting, expNumber};

        if tempOutput.success == 0 %if not able to reach goal
            successMetricTable(expSetting, expNumber) = 0;
        else
            successMetricTable(expSetting, expNumber) = mean(1-rolloutOutput.collided); %mean of no collisions
        end
        
        if isempty(rolloutOutput) %when exited in the funnel RRG construction phase itself (before moving)
            pathLengthMetricTable(expSetting, expNumber, :) = zeros(1,5);
            timeFlownMetricTable(expSetting, expNumber, :) = zeros(1,5);
        else
            pathLengthMetricTable(expSetting, expNumber, :) = quantile(rolloutOutput.distFlown, [0 0.1 0.5 0.9 1]);
            timeFlownMetricTable(expSetting, expNumber, :) = quantile(rolloutOutput.timeFlown, [0 0.1 0.5 0.9 1]);
        end
    end %trials loop
end %numObstacles loop

%% Print and plot statistical measures
switch expType
    case 'forest_sense'
        mean(successMetricTable,2)
        
        %figure; hold on
        mean(pathLengthMetricTable,2,"omitnan")
        %plot(median(pathLengthMetricTable,2,"omitnan"));
               
        %figure; hold on
        mean(timeFlownMetricTable,2,"omitnan")
        %plot(median(timeFlownMetricTable,2,"omitnan"));
    case 'maze_sense'
        fprintf("\nAverage success rate: %.2f", 100*mean(successMetricTable));
        fprintf("\nAverage of quantiles [0, 0.1, 0.5, 0.9, 1]:");
        
        fprintf("\nDistance flown in xy-plane (in m):");
        disp(squeeze(mean(pathLengthMetricTable,1, "omitnan"))');

        fprintf("Timeflown (in s):");
        disp(squeeze(mean(timeFlownMetricTable,1, "omitnan"))');
    otherwise
        error('Unsupported experiment type');
end
%% Saving printed metrics for later reference and paper writing

%% Maze environment with 50 trials..

% 1. RRT-X:
% Average success rate: 4.58
% Average of quantiles [0, 0.1, 0.5, 0.9, 1]
% Distance flown in xy-plane (in m):    9.7654   28.1147   44.1381   53.2367   66.1093
% Timeflown (in s):   14.9971   53.9581   82.5175   82.7582   83.4801
%
% 2. PiP-X:
% Average success rate: 71.54
% Average of quantiles [0, 0.1, 0.5, 0.9, 1]
% Distance flown in xy-plane (in m):   77.9999   78.0377   78.0936   79.9383   80.0086
% Timeflown (in s):  180.4443  180.4636  180.4822  184.7392  184.7392
%% Forest environment with 50 trials..
