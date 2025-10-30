clc; clearvars; close all;
%parpool %for parallel processing
%keyboard

if ~exist('./funnelLibrary/', 'dir')
    mkdir('funnelLibrary/');
end


if exist('./precomputedData/', 'dir')
    rmdir('./precomputedData/', 's');   %delete the folder if it already exists
end

addpath('./lib');

%instantiate an empty dictionary
%funnelLibrary = dictionary();  %for first run
%load('./library.mat');        %for subsequent runs

if ~exist('./library.mat', 'file')
    funnelLibrary = dictionary();  %for first run
else
    load('./library.mat');        %for subsequent runs
end

epsilon = 4;
resolution = 1; 

%finalXArray = -epsilon:resolution:epsilon;
%finalYArray = -epsilon:resolution:epsilon;

finalXArray = -4:1:4;
finalYArray = -4:1:4;

totalCount = length(finalXArray)*length(finalYArray);

%%

count = 0;

progressBar = waitbar(0, 'Funnel Library computation progress');
for y_iteration = 1:length(finalYArray)

    for x_iteration = 1:length(finalXArray)
        
        clearvars -except count totalCount funnelLibrary finalXArray finalYArray ... 
                          x_iteration y_iteration progressBar
        close all;
        
        mkdir('./precomputedData/');
        copyfile('./lib/taylorApprox_degree3.mat', './precomputedData/taylorApprox_degree3.mat');

        x_final = finalXArray(x_iteration);
        y_final = finalYArray(y_iteration);

        if x_final == 0 && y_final == 0
            continue
        end
        
        initialPose = [0; 0; 2; 0; 0; 0];   % initial state: origin at height of 2m with zero attitude
        finalPose   = [x_final; y_final; 2; 0; 0; 0];   % desired final pose

        %save the meta data of the funnel
        initialState = [initialPose(1:3); zeros(3,1); initialPose(4:6); zeros(3,1)];
        finalState   = [finalPose(1:3); zeros(3,1); finalPose(4:6); zeros(3,1)];
        save('./precomputedData/metaData.mat', 'initialState', 'finalState');

        run("main.m");
        %keyboard

        clc;
        count = count + 1;
        waitbar(count/totalCount)

        %Retrieve from precomputedData and save it as a
        %dictionary using the following function
        clearvars -except count totalCount funnelLibrary finalXArray finalYArray ...
                          x_iteration y_iteration progressBar
        saveToDictionary;

        % File handling commands
        copyfile('./precomputedData/', './funnelLibrary/temp/'); %copy it into the funnelLibrary folder
                                                         %and rename
        % rename the file inside funnelLibrary folder
        load('./funnelLibrary/temp/metaData.mat');
        fileName = "funnel_" + num2str(finalState(1)) + "," + num2str(finalState(2));
        fileNameWithDirectory = "funnelLibrary/" + fileName;
        movefile('./funnelLibrary/temp', fileNameWithDirectory);     %rename the folder to include meta data


        rmdir('./precomputedData/', 's');       %delete the folder and get it ready 
                                                %for the next computation        
        save('./library.mat','funnelLibrary');  %saving backup after each run
    end
end

close(progressBar);
clearvars -except totalCount funnelLibrary initialXArray initialYArray

%% Saving the funnel library
save('library.mat','funnelLibrary')

%% Plotting the funnel library: drawFunnelLibrary
%funnelList = values(funnelLibrary);

%each funnel has attributes .time, .trajectory and .RofA
%to look up funnel from dictionary: funnelLibrary(num2str([delta_x, delta_y]))

%for i=1:length(funnelList)
%    thisFunnel = funnelList(i);
%end

% thisFunnel = struct('time',NaN(1,N),'trajectory',NaN(6,N),'RofA',NaN(6,6,N));
