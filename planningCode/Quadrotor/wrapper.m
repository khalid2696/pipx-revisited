clc; clearvars; close all

numTrials = 5;

computationTime = NaN(1,numTrials); 
successArray = NaN(1,numTrials); 
lengthArray = NaN(1,numTrials);
replanningComputeTimeHistory = [];

expType = 'forest_sense';
numTreeObstacles = 50;

for expNumber = 1:numTrials
    
    expNumber

    clearvars -except expType expNumber numTrials numTreeObstacles ...
                 successArray lengthArray replanningComputeTimeHistory
    %close all; clc;
    
    try
        run('./PiPXForestSense.m');
    catch
        success = 0;
        traversedPathLength = NaN;
        replanningComputeTimes = [];
    end

    if success == 0
        traversedPathLength = NaN;
    end

    successArray(expNumber) = success;
    lengthArray(expNumber) = traversedPathLength;
    replanningComputeTimeHistory = [replanningComputeTimeHistory; replanningComputeTimes];
end

%% Post-processing

clearvars -except expType numTrials numTreeObstacles successArray lengthArray replanningComputeTimeHistory
    
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