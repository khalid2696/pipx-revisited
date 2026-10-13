clc; clearvars; close all
% return

%select experiment setting
%Options: 'forest_sense', 'forest_dynamic', 'forest_moving', 'maze_sense', 'maze_dynamic'
expType = 'forest_dynamic';
parentDir = fullfile('.', 'experiment_data', expType);

% Load the metadata and the experiment data 
load(fullfile(parentDir, 'experimentMetadata.mat'), ...
     'numTreeObstaclesArray', 'obstacleDynamicityArray', 'numTrials');

% Extract experiment output data
if ~exist(fullfile(parentDir, 'experimentData.mat'), 'file')
    disp('Extracting metrics and saving output data file...');
    run('./aggregateResults.m');
else
    disp('Loading experiment output data file..');
    load(fullfile(parentDir, 'experimentData.mat'), 'experimentsOutputTable');
end  

% Load the obstacle densities (~ ratio of vol(O)/vol(W)) file (for plotting)
% load('./utils/obstacleDensities_table.mat') 
% Double-verify the correspondence: numObstacles <--> density
load('./utils/obstacleDensities_randomForest.mat')

successMetricTable = NaN(size(experimentsOutputTable));
pathLengthMetricTable = NaN(size(experimentsOutputTable));

% experimentsOutputTable stores all the experimental data
for expSetting1 = 1:size(experimentsOutputTable,1) %numTreeObstacles
    % numTreeObstacles = numTreeObstaclesArray(expSetting1);
    
    for expSetting2 = 1:size(experimentsOutputTable,2)
        % obstacleDynamicity = obstacleDynamicityArray(expSetting2); %obstacleDynamicity
        
        for expNumber = 1:size(experimentsOutputTable,3) %numTrials
           
            tempOutput = experimentsOutputTable{expSetting1, expSetting2, expNumber}; 

            % tempOutput.success
            % tempOutput.funnelPathCost
            % tempOutput.traversedFunnelPath %isempty(traversedFunnelPath{index}) to figure out if the robot was idle
            % tempOutput.preplanningComputeTime
            % tempOutput.startFoundIterationCount
            % tempOutput.onlineReplanningComputeTimes
            % tempOutput.errorType

            successMetricTable(expSetting1, expSetting2, expNumber) = tempOutput.success;
            pathLengthMetricTable(expSetting1, expSetting2, expNumber) = tempOutput.funnelPathCost;

        end %MC trials loop
    end %obstacleDynamicity loop
end %numObstacles loop

%% Extracting statistical measures (mean/std_dev or median/quantiles) and plotting

% Adding plotting functions
addpath('./utils/plottingFns');

close all

switch expType
    case 'forest_sense'
        %instantiate an empty figure (for IJRR paper)
        fig = ijrr_fig('single', 1); %single column | ratio of height/width
        plot_layout  = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        
        %subplot 1
        meanSuccess = mean(successMetricTable, 3, "omitnan"); %along dimension 3 (random trials)
        % std_DevSuccess = std(successMetricTable, 0, 3, "omitnan"); %not relevant
        
        plot1 = nexttile(plot_layout);
        plot(numTreeObstaclesArray, meanSuccess, '-sb', 'LineWidth', 1.5, 'MarkerSize', 3, 'MarkerFaceColor', 'b');  

        % plot(obstacleDensities(1:numel(meanSuccess)),meanSuccess);
        ylabel(plot1, 'Success rate');
        xticks(0:10:100); ylim(plot1, [0 1.1]);
        xticklabels(plot1, {});                       % shared axis: label once
        
        %subplot 2
        % Remove Inf and -Inf before calculating the mean
        %pathLengthMetricTable = pathLengthMetricTable(~isinf(pathLengthMetricTable)); 
        pathLengthMetricTable(~isfinite(pathLengthMetricTable)) = NaN;   % failed or invalid trials out
        
        meanPathLength = mean(pathLengthMetricTable, 3, "omitnan"); %along dimension 3 (random trials)
        std_DevPathLength = std(pathLengthMetricTable, 0, 3, "omitnan");
        
        % alternatively quantiles
        q10 = quantile(pathLengthMetricTable, 0.1, 3);    % 10th percentile
        q90 = quantile(pathLengthMetricTable, 0.9, 3);    % 90th percentile
        spread_low = meanPathLength - q10;  % Distance down to the 0.1 quantile
        spread_high = q90 - meanPathLength; % Distance up to the 0.9 quantile
        
        plot2 = nexttile(plot_layout); hold on;
        yline(plot2, 40, '-.', 'LineWidth',1.5);
        %plot(numTreeObstaclesArray,meanPathLength);
        % errorbar(numTreeObstaclesArray, meanPathLength, std_DevPathLength, '-o', 'LineWidth', 1.5, 'MarkerSize', 6);
        %errorbar(numTreeObstaclesArray, meanPathLength, spread_low, spread_high, '-', 'LineWidth', 1.5, 'MarkerSize', 6); 
        
        x_conf = [numTreeObstaclesArray, fliplr(numTreeObstaclesArray)];
        y_conf = [q10', fliplr(q90')];

        fill(x_conf, y_conf, [0.8 0.9 1], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
        plot(numTreeObstaclesArray, meanPathLength, '-sb', 'LineWidth', 1.5, 'MarkerSize', 3, 'MarkerFaceColor', 'b');  

        % plot(obstacleDensities(1:numel(meanSuccess)),meanPathLength);
        ylabel(plot2, 'Traversed trajectory length [m]');
        ylim(plot2, [30 120]);
        xticks(0:10:100); yticks(40:20:120);
        xlabel(plot2, 'Number of obstacles, {\itN_t} ');
        
        %xlabel(ax, 'Time (s)', 'FontWeight', 'bold');
        
        legend(plot2, {'Nominal length (40 m)'}, 'Location', 'northeast');
        legend boxon;

        % shared properties
        linkaxes([plot1 plot2], 'x');
        xlim(plot1, [0 100]);
        
        % save figures (for IJRR paper) - png seems to be more stable (so use that)
        %args: fig handle, file name, format (png/eps), and folder to be saved
        ijrr_save(fig, 'forest_sense', 'png', Folder='./paper_figures');      
        
        %ijrr_save(fig, 'forest_sense', 'eps', Folder='./paper_figures');

    case 'forest_dynamic'
        fig = ijrr_fig('single', 1.25); %single column | ratio of height/width
        tl  = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        
        NO_DATA = [0.85 0.85 0.85];   % light gray = no successful trial
        
        % Metric 1: success rate
        plot1 = nexttile(tl);
        meanSuccess = mean(successMetricTable, 3, 'omitnan')';     % rows: C, cols: N_t
        plotHeatmap(plot1, numTreeObstaclesArray, obstacleDynamicityArray, meanSuccess, NO_DATA);
        clim(plot1, [0 1]);                                          % fixed: rate is in [0,1]
        cb1 = colorbar(plot1);
        cb1.Ticks = 0:0.2:1;
        title('Success rate');
        %cb1.Label.String = 'Success rate';
        xticklabels(plot1, {});                                      % shared x-axis
        
        % Metric 2: path length (successful trials only)
        plot2 = nexttile(tl);

        T = pathLengthMetricTable;
        T(~isfinite(T)) = NaN;   % failed or invalid trials out
        meanPathLength = mean(T, 3, 'omitnan')';

        %meanPathLength = mean(pathLengthMetricTable, 3, 'omitnan')';
        plotHeatmap(plot2, numTreeObstaclesArray, obstacleDynamicityArray, meanPathLength, NO_DATA);
        clim(plot2, [40 max(meanPathLength(isfinite(meanPathLength)))]);  % floor = nominal 40 m
        colormap(plot2, flipud(parula));        % short (good) = yellow
        clim(plot2, [40 100]);
        cb2 = colorbar(plot2);
        cb2.Direction = 'reverse';            % good at top, as in panel 1
        title('Traversed trajectory length [m]');
        xlabel(plot2, 'Number of obstacles, {\itN_t}');
        
        ylabel(tl, 'Change percentage, {\itC} [%]', ... % one label for both panels
                    'FontName', plot2.XLabel.FontName, 'FontSize', plot2.XLabel.FontSize);

        % Save
        ijrr_save(fig, 'forest_dynamic', 'png', Folder='./paper_figures');

    case 'maze_sense'
        fprintf('\nMaze environment with %d trials..\n', numTrials);
        fprintf('\nSuccess rate: %.2f percent',mean(successMetricTable, 3, "omitnan")*100);
        fprintf('\nMedian traversed trajectory length: %.2f m',median(pathLengthMetricTable, 3, "omitnan"));
        fprintf('\n(0.1,0.9) quantiles of traversed trajectory length: %.2f m, %0.2f m\n\n', ...
            quantile(pathLengthMetricTable, 0.1, 3), quantile(pathLengthMetricTable, 0.9, 3));
    case 'maze_dynamic'
        fprintf('\nSuccess rate for different change percentage (numTrials = %d):\n', numTrials);
        disp(obstacleDynamicityArray);
        disp(mean(successMetricTable, 3, "omitnan"));
    otherwise
        error('Unsupported experiment type');
end

%% Local function (must stay at the end of the script)
function plotHeatmap(ax, x, y, M, noDataColor)
    M(~isfinite(M)) = NaN;              % treat Inf like NaN
    h = imagesc(ax, x, y, M);
    h.AlphaData = ~isnan(M);            % NaN cells transparent ...
    ax.Color    = noDataColor;          % ... so the gray background shows
    ax.YDir     = 'normal';             % y(1) at bottom: C = 0 bottom, 50 top
    ax.XTick    = x(1:1:end);
    ax.YTick    = y;
    ax.TickDir  = 'out';                % ticks off the data
    ax.Box      = 'off';
    ax.Layer    = 'top';
end

% Processing to calculate computation times
% 
% replanningComputeTimeQuantiles = quantile(replanningComputeTimeHistory, [0.1, 0.5, 0.9]);
% 
% % Printing some metadata and results
% fprintf('\n\nEnvironment: %s, Number of obstacles: %d (%d trials)', expType, numTreeObstacles, numTrials);
% fprintf('\nSuccess rate                  -- mean: <strong>%0.2f</strong>, std dev: <strong>%0.2f</strong>', ...
%                     mean(successArray),std(successArray));
% fprintf('\nTraversed trajectory length -- mean: <strong>%0.2f</strong>, std dev: <strong>%0.2f</strong>', ...
%                     mean(lengthArray, 'omitnan'),std(lengthArray, 'omitnan'));
% fprintf('\nReplanning compute times -- median: <strong>%0.3f</strong> s, (0.1,0.9)-quant: (<strong>%0.3f</strong>, <strong>%0.3f</strong>) s', ...
%                     replanningComputeTimeQuantiles(2),replanningComputeTimeQuantiles(1),replanningComputeTimeQuantiles(3));
% fprintf('\nReplanning frquencies -- median: <strong>%0.2f</strong> Hz, (0.1,0.9)-quant: (<strong>%0.2f</strong>, <strong>%0.2f</strong>) Hz', ...
%                     1/replanningComputeTimeQuantiles(2),1/replanningComputeTimeQuantiles(1),1/replanningComputeTimeQuantiles(3));
% fprintf('\n\n');