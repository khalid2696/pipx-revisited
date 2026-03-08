clc; clearvars; close all

numExperiments = 20;

computationTime = NaN(1,numExperiments); 
successArray = NaN(1,numExperiments); 
lengthArray = NaN(1,numExperiments); 

numTreeObstacles = 60;

for expNumber = 1:numExperiments
    
    clearvars -except expNumber numExperiments successArray lengthArray numTreeObstacles
    close all; clc;

    expNumber
    
    %tic
    try
        run('./PiPXForestSense.m');
    catch
        success = 0;
        traversedPathLength = NaN;
    end
    %computationTime(expNumber) = toc;

    if success == 0
        traversedPathLength = NaN;
    end

    successArray(expNumber) = success;
    lengthArray(expNumber) = traversedPathLength;
end

%% Post-processing

clearvars -except numExperiments successArray lengthArray numTreeObstacles
    
numTreeObstacles

mean(successArray)
mean(lengthArray, 'omitnan')

std(successArray)
std(lengthArray, 'omitnan')

%mean(computationTime)