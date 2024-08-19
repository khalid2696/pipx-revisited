%% Mohamed Khalid M Jaffar (July '24)

% Requires installation of SPOT and Sedumi toolboxes (before running this file)

addpath('./lib');

clc
clearvars
close all

epsilon = 5;
theta = 2*pi*rand();

tspan = [0 10]; %0 25

x_initial = epsilon*cos(theta);
y_initial = epsilon*sin(theta);
z_initial = 0;

x0 = [x_initial y_initial z_initial 0 0 0]';
setpoint = 1e-3*ones(3,1);

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

%% plot the computed funnel

draw_funnel;
drawnow;


