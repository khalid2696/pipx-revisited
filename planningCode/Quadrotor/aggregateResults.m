% Aggregates all per-trial results into the master experimentsOutputTable.
% Run this once after (or even during) the experiments. Safe to re-run anytime.
clc; clearvars

expType = 'maze_dynamic';
parentDir = fullfile('.', 'experiment_data', expType);

load(fullfile(parentDir, 'experimentMetadata.mat'), ...
     'numTreeObstaclesArray', 'obstacleDynamicityArray', 'numTrials');

experimentsOutputTable = cell(numel(numTreeObstaclesArray), ...
                              numel(obstacleDynamicityArray), numTrials);
numMissing = 0;

for i = 1:numel(numTreeObstaclesArray)
    for j = 1:numel(obstacleDynamicityArray)
        for k = 1:numTrials
            resultFile = fullfile(parentDir, ...
                sprintf('obstacles_%g_dynamicity_%g', numTreeObstaclesArray(i), obstacleDynamicityArray(j)), ...
                sprintf('trial_%d', k), 'trialResult.mat');
            if isfile(resultFile)
                S = load(resultFile, 'expOutput');
                experimentsOutputTable{i,j,k} = S.expOutput;
            else
                numMissing = numMissing + 1;
            end
        end
    end
end

% Written once, at the end
save(fullfile(parentDir, 'experimentData.mat'), 'experimentsOutputTable', '-v7.3');

fprintf('Aggregated table saved. %d trial(s) missing/incomplete.\n', numMissing);