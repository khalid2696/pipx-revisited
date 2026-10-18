clc; clearvars; close all
% return

%adding paths to code libraries
addpath('./lib/');

%% Experiment settings
% options: 'forest_sense', 'maze_sense' 
expType = 'forest_sense';

parentDir = fullfile('.', 'experiment_data/baseline_comparison/PiPX/', expType);
% parentDir = fullfile('.', 'temp_invariance_verification', expType);

%% MC Rollouts

load(fullfile(parentDir, 'experimentMetadata.mat'), 'numTrials', 'numTreeObstaclesArray');

for expSetting1 = 1:numel(numTreeObstaclesArray)
    numTreeObstacles = numTreeObstaclesArray(expSetting1);

    for trialID = 1:numTrials
    
        clearvars -except parentDir trialID numTrials expSetting1 numTreeObstacles numTreeObstaclesArray
        
        fileSaveDir = fullfile(parentDir, ...
            sprintf('obstacles_%g', numTreeObstacles), sprintf('trial_%d', trialID));
        expOutputFile = fullfile(fileSaveDir, 'trialResult.mat');
        %Add this -- sprintf('obstacles_%g', numTreeObstacles) -- to previous line after parentDir (in case of forest_sense environment) 
        if isfile(expOutputFile)
            load(expOutputFile, 'expOutput', 'expDataStructures');
            
            % Run perturbations robustness script
            % Requires var names: 'funnelPath' and 'obstacles' (so extract from experiment output data)
            [funnelPath, obstacles] = postProcess(expOutput, expDataStructures);
    
            %script parameters
            num_samples = 100; n_perturb = 5; perturbLevel = 0.01;
            outputFile = fullfile('..', fileSaveDir, 'funnelPath_MCRollouts.mat');
            run('./utils/checkFunnelPath_Invariance_Perturbations.m');
        end
    end
end

%return
%% Aggregates all per-trial results into the master experimentsOutputTable and MCRolloutsOutputTable
% Run this once after (or even during) the experiments. Safe to re-run anytime.

fprintf("\nBuilding the master experimentsOutputTable.\n");

clearvars -except parentDir

load(fullfile(parentDir, 'experimentMetadata.mat'), 'numTrials', 'numTreeObstaclesArray');

experimentsOutputTable = cell(numel(numTreeObstaclesArray), numTrials);
MCRolloutsOutputTable = cell(numel(numTreeObstaclesArray), numTrials);
numMissing = 0;

for expSetting1 = 1:numel(numTreeObstaclesArray)
    numTreeObstacles = numTreeObstaclesArray(expSetting1);

    for trialID = 1:numTrials
        resultFile = fullfile(parentDir, sprintf('obstacles_%g', numTreeObstacles), sprintf('trial_%d', trialID), 'trialResult.mat');
        rolloutsResultFile = fullfile(parentDir, sprintf('obstacles_%g', numTreeObstacles), sprintf('trial_%d', trialID), 'funnelPath_MCRollouts.mat');
        if isfile(rolloutsResultFile) && isfile(resultFile)
            temp = load(rolloutsResultFile);
            MCRolloutsOutputTable{expSetting1, trialID} = temp.rollout;
            
            temp = load(resultFile, 'expOutput');
            experimentsOutputTable{expSetting1, trialID} = temp.expOutput;
        else
            numMissing = numMissing + 1;
        end
    end
end

% Written once, at the end
save(fullfile(parentDir, 'experimentData.mat'), 'experimentsOutputTable', 'MCRolloutsOutputTable', '-v7.3');

fprintf('Aggregated table saved. %d trial(s) missing/incomplete.\n', numMissing);

%% Function definitions

function [funnelPath, obstacles] = postProcess(expOutput, expDataStructures)
    F = expDataStructures.funnelRoadmap; G = expDataStructures.searchGraph;
    C = expDataStructures.configSpace; W = expDataStructures.workSpace;
    planner = expDataStructures.planner;

    % extracting the traversed funnel path and list of obstacles
    funnelPath = {};
    for i=1:numel(expOutput.traversedFunnelPath)
        tempFunnel = expOutput.traversedFunnelPath{i};
        if isempty(tempFunnel) %skip empty saved data (happens due to nature of file saving)
            continue
        end
        funnelPath{end+1} = tempFunnel;
        %F.drawFunnel(tempFunnel,2);
    end
    funnelPath = funnelPath';

    % Making a list of obstacles to collision check
    obstacles = NaN(numel(W.obstacles), 3); %3 - [xc yc r]
    for i=1:numel(W.obstacles)
        tempObstacle = W.obstacles{i};
        if tempObstacle.status == 0 %unsensed, so skip collision checking
            continue
        end
        obstacles(i,1:2) = tempObstacle.location;
        obstacles(i,3) = tempObstacle.radius;
    end
    obstacles = rmmissing(obstacles); %remove any inactive obstacles from the list

    % % visualizing traversed funnel path
    % planner.setupPlot(); W.drawAllObstacles();
    % 
    % for i=1:numel(funnelPath)
    %     tempFunnel = funnelPath{i};
    %     if isempty(tempFunnel) %skip empty saved data (happens due to nature of file saving)
    %         continue
    %     end
    %     F.drawFunnel(tempFunnel,2);
    % end
    % 
    % plot(planner.goalConfig(1), planner.goalConfig(2), 'xr', 'MarkerSize', 8, 'LineWidth', 3.5)
    % plot(planner.startConfig(1), planner.startConfig(2), 'sg', 'MarkerSize', 8, 'LineWidth', 3.5)
            
end