clc; clearvars; close all

%select experiment setting
expType = 'forest_sense';

%Load the experiment data
experimentsDataFilePath = ['./experiment_data/' expType '/experimentData.mat'];
load(experimentsDataFilePath);

% experimentsOutputTable stores all the experimental data
for expSetting1 = 1:size(experimentsOutputTable,1)

    numTreeObstacles = numTreeObstaclesArray(expSetting1);
    
    for expSetting2 = 1:size(experimentsOutputTable,2)
    
        obstacleDynamicity = obstacleDynamicityArray(expSetting2);

        for expNumber = 1:size(experimentsOutputTable,3)
            
            tempOutput = experimentsOutputTable{expSetting1, expSetting2, expNumber} 

            % tempOutput.success
            % tempOutput.funnelPathCost
            % tempOutput.traversedFunnelPath %isempty(traversedFunnelPath{index}) to figure out if the robot was idle
            % tempOutput.preplanningComputeTime
            % tempOutput.startFoundIterationCount
            % tempOutput.onlineReplanningComputeTimes
            % tempOutput.errorType
            
        end %MC trials loop
    end %obstacleDynamicity loop
end %numObstacles loop

fprintf("\n\n Extracted all %d trials of %d settings in '<strong>%s</strong>' environment\n", ...
                numTrials, numel(numTreeObstaclesArray)*numel(obstacleDynamicityArray), expType);
return
%% Post-processing

% clearvars -except expType numTrials numTreeObstacles successArray lengthArray replanningComputeTimeHistory traversedFunnelPath
    
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