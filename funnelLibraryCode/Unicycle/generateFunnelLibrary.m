clc; clearvars; close all;
%parpool %for parallel processing
keyboard

if ~exist('funnelLibrary/', 'dir')
    mkdir('funnelLibrary/')
end

initialState = [ 0; 0; pi/2;];   % initial state: at origin facing North
finalState   = [ 1; -4; pi/2;];   % desired final state

mkdir('./precomputedData/');
copyfile('./lib/taylorApprox_degree3.mat', './precomputedData/taylorApprox_degree3.mat');

%save the meta data of the funnel
save('./precomputedData/metaData.mat', 'initialState', 'finalState');

run("main.m");

%% File handling commands

copyfile('./precomputedData/', './funnelLibrary/temp/'); %copy it into the funnelLibrary folder
                                                         %and rename

%% rename the file inside funnelLibrary folder

load('./funnelLibrary/temp/metaData.mat');
fileName = "funnel_" + num2str(finalState(1)) + "," + num2str(finalState(2));
fileNameWithDirectory = "funnelLibrary/" + fileName;
movefile('./funnelLibrary/temp', fileNameWithDirectory);     %rename the folder to include meta data

rmdir('./precomputedData/', 's');                       %delete the folder and get it ready 
                                                        %for the next computation

%% Display progress
clc
disp('Progress: 1/16')
disp('- - - - - - -'); disp(" ");


% %% For saving it into dictionary format
% 
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