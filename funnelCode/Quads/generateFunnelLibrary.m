%% Mohamed Khalid M Jaffar (July '24)

% Requires installation of SPOT and Sedumi toolboxes (before running this file)

addpath('./lib');

clc
clearvars
%close all

%instantiate an empty dictionary
funnelLibrary = dictionary();

epsilon = 5;
resolution = 0.5; %1-Sparse %0.5-Nominal %0.25-Dense 

initialXArray = -epsilon:resolution:epsilon;
initialYArray = -epsilon:resolution:epsilon;

%%
tspan = [0 10]; %time span for the funnel
%count = 1;

for x_iteration =1:length(initialXArray)
    
    for y_iteration = 1:length(initialYArray)
        
        clearvars -except funnelLibrary tspan initialXArray initialYArray x_iteration y_iteration
        
        x_initial = initialXArray(x_iteration);
        y_initial = initialYArray(y_iteration);
        z_initial = 0;
        
        x0 = [x_initial y_initial z_initial 0 0 0]'
        setpoint = 1e-3*ones(3,1); %origin
        
        quad_example_setup;
        
        c = 3; %9
        rhot = exp(c*(taus-max(taus))/(max(taus)-min(taus)));
        rhopp = interp1(taus,rhot,'linear','pp'); %Resample data at finer level
        
        for iter=1:1
            lagrange_multipliers
            if ~all([gammas{:}] < 0)
                if iter == 1, error('Initial rho(t) infeasible, increase c.'); 
                end
            end
            improve_rho;
        end

        saveToDictionary;
    end
end

clearvars -except funnelLibrary tspan initialXArray initialYArray x_iteration y_iteration

%% Saving and plotting the funnel library
save('library.mat','funnelLibrary')
%drawFunnelLibrary