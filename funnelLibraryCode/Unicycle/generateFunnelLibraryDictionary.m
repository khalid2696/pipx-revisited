clc; clearvars; close all;
%parpool %for parallel processing

if ~exist('./funnelLibrary/', 'dir')
    mkdir('funnelLibrary/')
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

finalXArray = -1:1:1;
finalYArray = [-2 2];

totalCount = length(finalXArray)*length(finalYArray);
%%

count = 0;
progressBar = waitbar(0, 'Funnel Library computation progress');

for y_iteration = 1:length(finalYArray)

    for x_iteration = 1:length(finalXArray)
        
        clearvars -except count totalCount funnelLibrary finalXArray finalYArray x_iteration y_iteration progressBar
        
        mkdir('./precomputedData/');
        copyfile('./lib/taylorApprox_degree3.mat', './precomputedData/taylorApprox_degree3.mat');

        x_final = finalXArray(x_iteration);
        y_final = finalYArray(y_iteration);
        theta_final = pi/2;

        initialState = [ 0; 0; pi/2;];   % initial state: at origin facing North
        finalState   = [x_final; y_final; theta_final];    % desired final state

        %save the meta data of the funnel
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

        save('library.mat','funnelLibrary');

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
    end
end

close(progressBar);
clearvars -except totalCount funnelLibrary initialXArray initialYArray


%% Saving the funnel library
save('library.mat','funnelLibrary')

%% Plotting the funnel library: drawFunnelLibrary
load('library.mat')
addpath('./lib');

close all

funnelList = values(funnelLibrary);

figure; hold on; grid on; axis equal;

for i=1:10
   thisFunnel = funnelList(i);
   plotFunnel_2D(thisFunnel.trajectory, thisFunnel.RofA, [1 2]);
end

%%
function plotFunnel_2D(x_nom, ellipsoidMatrix, projectionDims)

    P = plottingFnsClass();
    
    if nargin < 4
        projectionDims = [1 2]; %if not specified, by default x-y projection
    end

    %plot ellipsoidal invariant sets in 2D
    for k=1:1:size(x_nom,2)
        M = ellipsoidMatrix(:,:,k);
        M_xy = P.project_ellipsoid_matrix_2D(M, projectionDims);
        center = [x_nom(projectionDims(1),k), x_nom(projectionDims(2),k)]';
        P.plotEllipse(center, M_xy);
    end 
    
    %nominal trajectory
    plot(x_nom(projectionDims(1),:),x_nom(projectionDims(2),:),'--b');
    
    plot(x_nom(projectionDims(1),end), x_nom(projectionDims(2),end), 'xr', 'MarkerSize', 8, 'LineWidth', 3.5)
    plot(x_nom(projectionDims(1),1), x_nom(projectionDims(2),1), 'sg', 'MarkerSize', 8, 'LineWidth', 3.5)

    %formatting
    xlabel('p_x');    ylabel('p_y');
    %xlabel(['x_{', num2str(projectionDims(1)), '}'])
    %ylabel(['x_{', num2str(projectionDims(2)), '}'])
end


%%
% funnel = struct('time',NaN(1,N),'trajectory',NaN(6,N),'RofA',NaN(6,6,N));
% 
% %Saving the data
% for j = 1:1:N-1
%     %Pi comes from Lagrange multiplier.m - substitute t in all polynomials with 0 
%     St = double(reshape(subs(Pi(:,j),t,0),n,n)); %reshape into n-by-n matrix (n - dimension of system)
% 
%     %x0i comes from Lagrange multiplier.m - substitute t in all polynomials with 0
%     xt = double(subs(x0i(:,j),t,0));
% 
%     ellipsoid = St./ppval(rhopp,ts(j));
% 
%     funnel.time(j) = ts(j);
%     funnel.trajectory(:,j) = xt;
%     funnel.RofA(:,:,j) = ellipsoid;
% end
% 
% funnel.time(N) = ts(N);
% funnel.trajectory(:,N) = x0s(end,:); %size of x0s is not equal to N, so use 'end'
% funnel.RofA(:,:,N) = S0;
% 
% % Now save it into the dictionary
% funnelLibrary(num2str([x_initial, y_initial])) = funnel;
