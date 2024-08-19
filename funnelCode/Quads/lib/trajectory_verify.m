%% Verifying whether trajectories lie within funnels

u0 = @(t) setpoint;
x_final = x0s(end,:)';

for i = 20:5:length(t0s)-5
    %specify the initial conditions
    delta_x = -0.5 + 1*rand(1,3);
    delta_v = -0.1 + 0.2*rand(1,3);
    x_initial = x0s(i,:) + [delta_x, delta_v];

    time_interval = [t0s(i) t0s(end)];
    
    %integrating the system dynamics forward
    [ts,xs] = ode45(@(t,x)f0(t,x,u0(t)),time_interval,x_initial);
    
    xf = xs(end,:)';
    if (xf-x_final)'*S0*(xf-x_final) > 1
        t0s(i)
        %error('Trajectory has gone out of the funnel');
    end
    plot2DTrajectory(xs,ts)
end

%function definitions
function plot2DTrajectory(x,t)
hold on
plot3(x(:,1),x(:,2),t,'-.m','LineWidth',2);

%Plotting the end and start points
plot3(x(1,1),x(1,2),t(1),'om','LineWidth',3,'MarkerSize',7);
plot3(x(end,1),x(end,2),t(end),'rx','LineWidth',3,'MarkerSize',7);

end