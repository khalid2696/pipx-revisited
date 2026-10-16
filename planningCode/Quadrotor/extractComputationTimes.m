clc; clearvars; close all
% return

%select experiment setting
%Options: 'forest_sense', 'maze_sense'
expType = 'maze_sense';
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
     numTreeObstacles = numTreeObstaclesArray(expSetting1);
    
    for expSetting2 = 1:size(experimentsOutputTable,2)
         obstacleDynamicity = obstacleDynamicityArray(expSetting2); %obstacleDynamicity
        
        replanningComputeTimes = NaN(size(experimentsOutputTable,3)*50,1); %pre-allocating for speed (assuming atmost 50 funnel-edges in funnel-path)
        countIndex = 1;
        for expNumber = 1:size(experimentsOutputTable,3) %numTrials
           
            tempOutput = experimentsOutputTable{expSetting1, expSetting2, expNumber}; 
            
            n = numel(tempOutput.onlineReplanningComputeTimes);
            replanningComputeTimes(countIndex:countIndex+n-1, 1) = tempOutput.onlineReplanningComputeTimes;
            countIndex = countIndex + n;
        end %MC trials loop

        replanningComputeTimes = replanningComputeTimes(1:countIndex-1);
        switch expType
            case 'forest_sense'
                % replanningComputeTimes = rmoutliers(replanningComputeTimes, "percentiles", [5, 95]); %optionally remove extreme outliers
                replanningComputeTimeQuantiles = quantile(replanningComputeTimes, [0.1, 0.25, 0.5, 0.75, 0.9]);
                fprintf('\nForest environment with %d obstacles %d trials..\n', numTreeObstacles, numTrials);
                fprintf('\nNumber of data points: %d', numel(replanningComputeTimes));
                fprintf(['\nReplanning compute times: \nMedian: %.3f ms ' ...
                            '\n(0.1,0.9) quantiles: %.3f ms, %0.3f ms ' ...
                            '\n(0.25,0.75) quantiles: %.3f ms, %0.3f ms'], ...
                            1000*replanningComputeTimeQuantiles(3), 1000*replanningComputeTimeQuantiles(1), ...
                            1000*replanningComputeTimeQuantiles(5), 1000*replanningComputeTimeQuantiles(2), 1000*replanningComputeTimeQuantiles(4));
                fprintf(['\nReplanning frequencies: \nMedian: %.3f Hz ' ...
                            '\n(0.1,0.9) quantiles: %.3f Hz, %0.3f Hz ' ...
                            '\n(0.25,0.75) quantiles: %.3f Hz, %0.3f Hz\n'], ...
                                1/replanningComputeTimeQuantiles(3), 1/replanningComputeTimeQuantiles(1), ...
                                1/replanningComputeTimeQuantiles(5), 1/replanningComputeTimeQuantiles(2), 1/replanningComputeTimeQuantiles(4));
            case 'maze_sense'
                % replanningComputeTimes = rmoutliers(replanningComputeTimes, "percentiles", [5, 95]); %optionally remove extreme outliers
                replanningComputeTimeQuantiles = quantile(replanningComputeTimes, [0.1, 0.25, 0.5, 0.75, 0.9]);
                fprintf('\nMaze environment with %d trials..\n', numTrials);
                fprintf('\nNumber of data points: %d', numel(replanningComputeTimes));
                fprintf(['\nReplanning compute times: \nMedian: %.3f ms ' ...
                            '\n(0.1,0.9) quantiles: %.3f ms, %0.3f ms ' ...
                            '\n(0.25,0.75) quantiles: %.3f ms, %0.3f ms'], ...
                            1000*replanningComputeTimeQuantiles(3), 1000*replanningComputeTimeQuantiles(1), ...
                            1000*replanningComputeTimeQuantiles(5), 1000*replanningComputeTimeQuantiles(2), 1000*replanningComputeTimeQuantiles(4));
                fprintf(['\nReplanning frequencies: \nMedian: %.3f Hz ' ...
                            '\n(0.1,0.9) quantiles: %.3f Hz, %0.3f Hz ' ...
                            '\n(0.25,0.75) quantiles: %.3f Hz, %0.3f Hz\n'], ...
                                1/replanningComputeTimeQuantiles(3), 1/replanningComputeTimeQuantiles(1), ...
                                1/replanningComputeTimeQuantiles(5), 1/replanningComputeTimeQuantiles(2), 1/replanningComputeTimeQuantiles(4));
            otherwise
                error('Unsupported experiment type');
        end
    end %obstacleDynamicity loop
end %numObstacles loop

%% Saving printed metrics for later reference and paper writing

%% Maze environment with 250 trials..
% 
% Number of data points: 6866
% Replanning compute times: 
% Median: 16.783 ms 
% (0.1,0.9) quantiles: 0.437 ms, 4602.182 ms 
% (0.25,0.75) quantiles: 14.545 ms, 80.771 ms
% Replanning frequencies: 
% Median: 59.584 Hz 
% (0.1,0.9) quantiles: 2288.330 Hz, 0.217 Hz 
% (0.25,0.75) quantiles: 68.752 Hz, 12.381 Hz

%% Forest environment with 0 obstacles 25 trials..
% 
% Number of data points: 291
% Replanning compute times: 
% Median: 8.479 ms 
% (0.1,0.9) quantiles: 6.307 ms, 12.121 ms 
% (0.25,0.75) quantiles: 7.167 ms, 10.428 ms
% Replanning frequencies: 
% Median: 117.938 Hz 
% (0.1,0.9) quantiles: 158.544 Hz, 82.501 Hz 
% (0.25,0.75) quantiles: 139.538 Hz, 95.891 Hz
% 
% Forest environment with 5 obstacles 25 trials..
% 
% Number of data points: 308
% Replanning compute times: 
% Median: 8.750 ms 
% (0.1,0.9) quantiles: 5.993 ms, 999.921 ms 
% (0.25,0.75) quantiles: 7.106 ms, 11.262 ms
% Replanning frequencies: 
% Median: 114.279 Hz 
% (0.1,0.9) quantiles: 166.867 Hz, 1.000 Hz 
% (0.25,0.75) quantiles: 140.736 Hz, 88.790 Hz
% 
% Forest environment with 10 obstacles 25 trials..
% 
% Number of data points: 309
% Replanning compute times: 
% Median: 8.587 ms 
% (0.1,0.9) quantiles: 6.152 ms, 1673.256 ms 
% (0.25,0.75) quantiles: 7.132 ms, 14.607 ms
% Replanning frequencies: 
% Median: 116.455 Hz 
% (0.1,0.9) quantiles: 162.543 Hz, 0.598 Hz 
% (0.25,0.75) quantiles: 140.218 Hz, 68.458 Hz
% 
% Forest environment with 15 obstacles 25 trials..
% 
% Number of data points: 321
% Replanning compute times: 
% Median: 7.864 ms 
% (0.1,0.9) quantiles: 5.834 ms, 1566.647 ms 
% (0.25,0.75) quantiles: 6.541 ms, 158.795 ms
% Replanning frequencies: 
% Median: 127.162 Hz 
% (0.1,0.9) quantiles: 171.409 Hz, 0.638 Hz 
% (0.25,0.75) quantiles: 152.876 Hz, 6.297 Hz
% 
% Forest environment with 20 obstacles 25 trials..
% 
% Number of data points: 323
% Replanning compute times: 
% Median: 7.467 ms 
% (0.1,0.9) quantiles: 5.690 ms, 1507.634 ms 
% (0.25,0.75) quantiles: 6.282 ms, 213.414 ms
% Replanning frequencies: 
% Median: 133.923 Hz 
% (0.1,0.9) quantiles: 175.747 Hz, 0.663 Hz 
% (0.25,0.75) quantiles: 159.191 Hz, 4.686 Hz
% 
% Forest environment with 25 obstacles 25 trials..
% 
% Number of data points: 365
% Replanning compute times: 
% Median: 7.867 ms 
% (0.1,0.9) quantiles: 5.576 ms, 1429.735 ms 
% (0.25,0.75) quantiles: 6.278 ms, 284.835 ms
% Replanning frequencies: 
% Median: 127.113 Hz 
% (0.1,0.9) quantiles: 179.340 Hz, 0.699 Hz 
% (0.25,0.75) quantiles: 159.280 Hz, 3.511 Hz
% 
% Forest environment with 30 obstacles 25 trials..
% 
% Number of data points: 409
% Replanning compute times: 
% Median: 7.410 ms 
% (0.1,0.9) quantiles: 0.253 ms, 1812.886 ms 
% (0.25,0.75) quantiles: 5.855 ms, 195.886 ms
% Replanning frequencies: 
% Median: 134.953 Hz 
% (0.1,0.9) quantiles: 3958.828 Hz, 0.552 Hz 
% (0.25,0.75) quantiles: 170.787 Hz, 5.105 Hz
% 
% Forest environment with 35 obstacles 25 trials..
% 
% Number of data points: 357
% Replanning compute times: 
% Median: 7.404 ms 
% (0.1,0.9) quantiles: 0.320 ms, 1700.769 ms 
% (0.25,0.75) quantiles: 6.084 ms, 416.813 ms
% Replanning frequencies: 
% Median: 135.062 Hz 
% (0.1,0.9) quantiles: 3123.048 Hz, 0.588 Hz 
% (0.25,0.75) quantiles: 164.372 Hz, 2.399 Hz
% 
% Forest environment with 40 obstacles 25 trials..
% 
% Number of data points: 360
% Replanning compute times: 
% Median: 6.689 ms 
% (0.1,0.9) quantiles: 0.199 ms, 1273.960 ms 
% (0.25,0.75) quantiles: 0.391 ms, 80.494 ms
% Replanning frequencies: 
% Median: 149.488 Hz 
% (0.1,0.9) quantiles: 5025.126 Hz, 0.785 Hz 
% (0.25,0.75) quantiles: 2560.819 Hz, 12.423 Hz
% 
% Forest environment with 45 obstacles 25 trials..
% 
% Number of data points: 497
% Replanning compute times: 
% Median: 6.992 ms 
% (0.1,0.9) quantiles: 0.210 ms, 1455.256 ms 
% (0.25,0.75) quantiles: 0.474 ms, 252.083 ms
% Replanning frequencies: 
% Median: 143.021 Hz 
% (0.1,0.9) quantiles: 4766.444 Hz, 0.687 Hz 
% (0.25,0.75) quantiles: 2109.705 Hz, 3.967 Hz
% 
% Forest environment with 50 obstacles 25 trials..
% 
% Number of data points: 449
% Replanning compute times: 
% Median: 6.852 ms 
% (0.1,0.9) quantiles: 0.194 ms, 1477.560 ms 
% (0.25,0.75) quantiles: 5.239 ms, 243.643 ms
% Replanning frequencies: 
% Median: 145.943 Hz 
% (0.1,0.9) quantiles: 5159.959 Hz, 0.677 Hz 
% (0.25,0.75) quantiles: 190.885 Hz, 4.104 Hz
% 
% Forest environment with 55 obstacles 25 trials..
% 
% Number of data points: 441
% Replanning compute times: 
% Median: 6.260 ms 
% (0.1,0.9) quantiles: 0.161 ms, 1104.393 ms 
% (0.25,0.75) quantiles: 0.263 ms, 15.869 ms
% Replanning frequencies: 
% Median: 159.744 Hz 
% (0.1,0.9) quantiles: 6211.180 Hz, 0.905 Hz 
% (0.25,0.75) quantiles: 3802.281 Hz, 63.014 Hz
% 
% Forest environment with 60 obstacles 25 trials..
% 
% Number of data points: 426
% Replanning compute times: 
% Median: 5.914 ms 
% (0.1,0.9) quantiles: 0.190 ms, 1016.418 ms 
% (0.25,0.75) quantiles: 0.253 ms, 163.969 ms
% Replanning frequencies: 
% Median: 169.090 Hz 
% (0.1,0.9) quantiles: 5263.158 Hz, 0.984 Hz 
% (0.25,0.75) quantiles: 3952.569 Hz, 6.099 Hz
% 
% Forest environment with 65 obstacles 25 trials..
% 
% Number of data points: 418
% Replanning compute times: 
% Median: 5.907 ms 
% (0.1,0.9) quantiles: 0.184 ms, 1121.593 ms 
% (0.25,0.75) quantiles: 0.330 ms, 59.443 ms
% Replanning frequencies: 
% Median: 169.305 Hz 
% (0.1,0.9) quantiles: 5449.591 Hz, 0.892 Hz 
% (0.25,0.75) quantiles: 3030.303 Hz, 16.823 Hz
% 
% Forest environment with 70 obstacles 25 trials..
% 
% Number of data points: 393
% Replanning compute times: 
% Median: 0.598 ms 
% (0.1,0.9) quantiles: 0.164 ms, 663.818 ms 
% (0.25,0.75) quantiles: 0.225 ms, 7.279 ms
% Replanning frequencies: 
% Median: 1672.241 Hz 
% (0.1,0.9) quantiles: 6105.006 Hz, 1.506 Hz 
% (0.25,0.75) quantiles: 4454.343 Hz, 137.391 Hz
% 
% Forest environment with 75 obstacles 25 trials..
% 
% Number of data points: 407
% Replanning compute times: 
% Median: 5.928 ms 
% (0.1,0.9) quantiles: 0.155 ms, 930.473 ms 
% (0.25,0.75) quantiles: 0.199 ms, 84.021 ms
% Replanning frequencies: 
% Median: 168.691 Hz 
% (0.1,0.9) quantiles: 6443.299 Hz, 1.075 Hz 
% (0.25,0.75) quantiles: 5025.126 Hz, 11.902 Hz
% 
% Forest environment with 80 obstacles 25 trials..
% 
% Number of data points: 367
% Replanning compute times: 
% Median: 5.353 ms 
% (0.1,0.9) quantiles: 0.136 ms, 721.795 ms 
% (0.25,0.75) quantiles: 0.194 ms, 11.531 ms
% Replanning frequencies: 
% Median: 186.811 Hz 
% (0.1,0.9) quantiles: 7352.941 Hz, 1.385 Hz 
% (0.25,0.75) quantiles: 5148.005 Hz, 86.721 Hz
% 
% Forest environment with 85 obstacles 25 trials..
% 
% Number of data points: 362
% Replanning compute times: 
% Median: 0.439 ms 
% (0.1,0.9) quantiles: 0.131 ms, 566.698 ms 
% (0.25,0.75) quantiles: 0.188 ms, 6.970 ms
% Replanning frequencies: 
% Median: 2280.502 Hz 
% (0.1,0.9) quantiles: 7610.350 Hz, 1.765 Hz 
% (0.25,0.75) quantiles: 5319.149 Hz, 143.472 Hz
% 
% Forest environment with 90 obstacles 25 trials..
% 
% Number of data points: 314
% Replanning compute times: 
% Median: 0.257 ms 
% (0.1,0.9) quantiles: 0.134 ms, 571.967 ms 
% (0.25,0.75) quantiles: 0.181 ms, 6.294 ms
% Replanning frequencies: 
% Median: 3898.635 Hz 
% (0.1,0.9) quantiles: 7468.260 Hz, 1.748 Hz 
% (0.25,0.75) quantiles: 5524.862 Hz, 158.881 Hz
% 
% Forest environment with 95 obstacles 25 trials..
% 
% Number of data points: 328
% Replanning compute times: 
% Median: 0.272 ms 
% (0.1,0.9) quantiles: 0.116 ms, 372.417 ms 
% (0.25,0.75) quantiles: 0.163 ms, 6.324 ms
% Replanning frequencies: 
% Median: 3676.471 Hz 
% (0.1,0.9) quantiles: 8620.690 Hz, 2.685 Hz 
% (0.25,0.75) quantiles: 6134.969 Hz, 158.128 Hz
% 
% Forest environment with 100 obstacles 25 trials..
% 
% Number of data points: 273
% Replanning compute times: 
% Median: 0.261 ms 
% (0.1,0.9) quantiles: 0.137 ms, 607.512 ms 
% (0.25,0.75) quantiles: 0.177 ms, 6.575 ms
% Replanning frequencies: 
% Median: 3831.418 Hz 
% (0.1,0.9) quantiles: 7309.942 Hz, 1.646 Hz 
% (0.25,0.75) quantiles: 5665.722 Hz, 152.085 Hz