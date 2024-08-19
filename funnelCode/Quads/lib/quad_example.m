clc
clearvars
%close all


%specify the initial conditions
%initial = [zeros(1,6); [3 5 0 0 0 0]; [-4 4 0 0 0 0];];
%final = [[5 0 0]; [4 3 0]; [0 4 0]; [-3 4 0]; [-4 0 0]; [-4 -3 0]; [0 -5 0]; [3 -4 0];];
library = containers.Map('KeyType','single','ValueType','any');
save('funnelLibrary.mat');
%specify the setpoint

%setpoint = [-4;4; 0];
%setpoint = [-5;-2; 0];
%setpoint = [1;-6; 0];

%time_input = [[0 25];[0 25];[0 25];]; % duration of input

figure
hold on
view(3)
grid on
%axis equal
daspect([1 1 2]) %maintains aspect (scaling)
set(gca, 'ZDir','reverse') %invert the direction of z
hold on
xlabel('x');
ylabel('y');
zlabel('time');

epsilon = 5;
theta = linspace(0,2*pi,13); %13-Sparse %25-Nominal %37-Dense 

for count = 5:5%length(theta)-1
    
    %x0 = zeros(1,6)';
    tspan = [0 10]; %0 25
    x_initial = epsilon*cos(theta(count))
    y_initial = epsilon*sin(theta(count))
    z_initial = 0;
    
    x0 = [x_initial y_initial z_initial 0 0 0]';
    setpoint = 1e-3*ones(3,1);

    %x0 = zeros(1,6)';
    %setpoint = [x_setpoint y_setpoint z_setpoint]';
    
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
        %plot(ts,ppval(rhopp,ts)); drawnow;
        %ppval evaluates the piecewise polynomial at the given time
        improve_rho;
        %plot(ts,ppval(rhopp,ts)); drawnow;
end

%%
%plot_quad;
quad_saveEdited;
drawnow;
%trajectory_verify;
%quad_save;
end

