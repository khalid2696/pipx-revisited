% clc; close all; 
clearvars
% return

%% Experiment settings
% options: 'forest_sense', 'maze_sense' 
expType = 'forest_sense';

parentDir = fullfile('.', 'experiment_data/baseline_comparison/PiPX', expType);

% Load the metadata and the experiment data
% load(fullfile(parentDir, 'experimentMetadata.mat'));

% Extract experiment output data
disp('Loading experiment output data file..');
load(fullfile(parentDir, 'experimentData.mat'), 'experimentsOutputTable', 'MCRolloutsOutputTable');

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
        
        successRate = mean(successMetricTable,2);
        fprintf("\nAverage success rate:\n");
        disp(100*successRate);

        pathLengthAggregate = squeeze(mean(pathLengthMetricTable,2,"omitnan"));
        fprintf("\nDistance flown in xy-plane (in m):");
        disp(pathLengthAggregate);
         
        timeFlownAggregate = squeeze(mean(timeFlownMetricTable,2,"omitnan"));
        fprintf("Time flown (in s):");
        disp(timeFlownAggregate);

        pipxResults = struct();
        pipxResults.successRate = successRate;
        pipxResults.pathLengthAggregate = pathLengthAggregate;
        pipxResults.timeFlownAggregate = timeFlownAggregate;
    case 'maze_sense'
        fprintf("\nAverage success rate: %.2f", 100*mean(successMetricTable));
        fprintf("\nAverage of quantiles [0, 0.1, 0.5, 0.9, 1]:");
        
        fprintf("\nDistance flown in xy-plane (in m):");
        disp(squeeze(mean(pathLengthMetricTable,1, "omitnan"))');

        fprintf("Time flown (in s):");
        disp(squeeze(mean(timeFlownMetricTable,1, "omitnan"))');
    otherwise
        error('Unsupported experiment type');
end

return

%% Loading extracted data for paper plotting

addpath('./utils/plottingFns');
close all; clearvars

% Following loads two structs: pipxResults and rrtxResults
% Each have the following fields: successRate, pathLengthAggregate, timeFlownAggregate
load('./experiment_data/baseline_comparison/aggregates_forestsense.mat');

% Load the obstacle densities (~ ratio of vol(O)/vol(W)) file (for plotting)
load('./utils/obstacleDensities_randomForest.mat', 'obstacleDensities');

numObstacleSettings = numel(pipxResults.successRate);
x = obstacleDensities(2:numObstacleSettings+1);   % skip 0% density
x = x(:);                                         % force column

% Colors: Okabe-Ito blue / vermillion (colorblind-safe); markers differ for grayscale
C_PIPX     = [0.00 0.45 0.70];
C_RRTX     = [0.84 0.37 0.00];
BAND_ALPHA = 0.3;                            

%instantiate an empty figure (for IJRR paper)
fig = ijrr_fig('single', 1.1);
tl  = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% Panel 1: success rate
ax1 = nexttile(tl); hold(ax1, 'on');
plot(ax1, x, pipxResults.successRate, '-s', 'Color', C_PIPX, ...
    'MarkerFaceColor', C_PIPX, 'MarkerSize', 3, 'DisplayName', 'PiP-X');
plot(ax1, x, rrtxResults.successRate, '-o', 'Color', C_RRTX, ...
    'MarkerFaceColor', C_RRTX, 'MarkerSize', 3, 'DisplayName', 'RRT^X');
ylabel(ax1, 'Success rate');
ylim(ax1, [0 1.1]);
xticklabels(ax1, {});                             % shared x-axis
legend(ax1, 'Location', 'northeast');

% Panel 2: path length, median with 10-90% quantile band
% pathLengthAggregate columns: [q0 q0.1 q0.5 q0.9 q1]
ax2 = nexttile(tl); hold(ax2, 'on');
yline(ax2, 40, '--', 'LineWidth',1);
%0.1,0.9 quantiles
plotBand(ax2, x, pipxResults.pathLengthAggregate, C_PIPX, BAND_ALPHA);
plotBand(ax2, x, rrtxResults.pathLengthAggregate, C_RRTX, BAND_ALPHA);   % bands first
%medians
plot(ax2, x, pipxResults.pathLengthAggregate(:, 3), '-s', 'Color', C_PIPX, ...
    'MarkerFaceColor', C_PIPX, 'MarkerSize', 3, 'HandleVisibility', 'off');
plot(ax2, x, rrtxResults.pathLengthAggregate(:, 3), '-o', 'Color', C_RRTX, ...
    'MarkerFaceColor', C_RRTX, 'MarkerSize', 3, 'HandleVisibility', 'off');

xticklabels(ax2, {});  
ylabel(ax2, 'Path length [m]');
legend(ax2, {'Start to Goal distance'}, 'Location', 'southwest');

% Panel 3: time flown, median with 10-90% quantile band
% timeFlownAggregate columns: [q0 q0.1 q0.5 q0.9 q1]
ax3 = nexttile(tl); hold(ax3, 'on');
%0.1,0.9 quantiles
plotBand(ax3, x, pipxResults.timeFlownAggregate, C_PIPX, BAND_ALPHA);
plotBand(ax3, x, rrtxResults.timeFlownAggregate, C_RRTX, BAND_ALPHA);   % bands first
%medians
plot(ax3, x, pipxResults.timeFlownAggregate(:, 3), '-s', 'Color', C_PIPX, ...
    'MarkerFaceColor', C_PIPX, 'MarkerSize', 3, 'HandleVisibility', 'off');
plot(ax3, x, rrtxResults.timeFlownAggregate(:, 3), '-o', 'Color', C_RRTX, ...
    'MarkerFaceColor', C_RRTX, 'MarkerSize', 3, 'HandleVisibility', 'off');
ylabel(ax3, 'Time flown [s]'); ylim(ax3, [20 150]); yticks(ax3, 25:25:150);

linkaxes([ax1 ax2 ax3], 'x');
xlabel(ax3, 'Obstacle density [%]');

% Save
ijrr_save(fig, 'baseline_comparisons', 'png', Folder='./paper_figures');

%% Local function
function plotBand(ax, x, Q, color, alpha)
    lo = Q(:, 2);  hi = Q(:, 4); %0.1 and 0.9 quantiles
    ok = isfinite(lo) & isfinite(hi);                 % skip settings with no successes
    xb = x(ok);  lo = lo(ok);  hi = hi(ok);
    fill(ax, [xb; flipud(xb)], [lo; flipud(hi)], color, ...
        'FaceAlpha', alpha, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end

%% Saving printed metrics for later reference and paper writing

%% Maze environment with 50 trials..

% 1. RRT-X:
% Average success rate: 4.58
% Average of quantiles [0, 0.1, 0.5, 0.9, 1]
% Distance flown in xy-plane (in m):    9.7654   28.1147   44.1381   53.2367   66.1093
% Time flown (in s):   14.9971   53.9581   82.5175   82.7582   83.4801
%
% 2. PiP-X:
% Average success rate: 71.54
% Average of quantiles [0, 0.1, 0.5, 0.9, 1]
% Distance flown in xy-plane (in m):   77.9999   78.0377   78.0936   79.9383   80.0086
% Time flown (in s):  180.4443  180.4636  180.4822  184.7392  184.7392
%% Forest environment with 50 trials..

% Results stored in two structs: pipxResults and rrtxResults
% Each have the following fields: successRate, pathLengthAggregate, timeFlownAggregate
% load('./experiment_data/baseline_comparison/aggregates_forestsense.mat');