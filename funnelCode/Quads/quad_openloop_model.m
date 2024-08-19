%% Disable Warning

warning('off','MATLAB:nearlySingularMatrix');
%close all
clearvars
clc

%initial and final state definitions
epsilon = 5;
theta = 30;

tspan = [0 10]; %0 25
x_initial = epsilon*cosd(theta)
y_initial = epsilon*sind(theta)
z_initial = 0;

x0 = [x_initial y_initial z_initial 0 0 0]';
setpoint = 1e-3*ones(3,1);

%% Define the Dynamics
n=6;   % Number of States
m=3; % Number of Inputs
 
%b = 2*zeta*omega_n
%c = omega_n^2

%settling time -- 4/(zeta*omega_n) = 8/b
%5/(zeta*omega_n) seems to be the right one, so b = 10/Ts

%values for settling time = 5 seconds, zeta*omega_n = 0.8

%critically damped implies zeta = 1, c = b^2/4
%for Ts = 5, b = 1.6, c = 0.64 (critically damped)

%zeta = 0.9: b = 1.6, c = 0.79 (underdamped)

%zeta = 0.8: b = 1.6, c = 1 (underdamped)

%zeta = 1.1: b = 1.6; c = 0.53 (over-damped)
% c/(s^2 + bs + c)
b_maxDisturbance = 0.05;
c_maxDisturbance = 0.05;

b1 = 1.85 + b_maxDisturbance*rand(); %2.05
c1 = 0.56 + c_maxDisturbance*rand(); %0.96

b2 = 1.65 + b_maxDisturbance*rand();
c2 = 0.62 + c_maxDisturbance*rand();

b3 = 0.2 + b_maxDisturbance*rand();
c3 = 0.0411 + c_maxDisturbance*rand();

% b1 = 0.3133;
% c1 = 0.0543;
% 
% b2 = 0.3033;
% c2 = 0.0416;
% 
% b3 = 0.2;
% c3 = 0.0411;
f0 = @(t,x,u) [    x(4); x(5); x(6); 
               -c1*x(1)-b1*x(4)+c1*u(1); 
               -c2*x(2)-b2*x(5)+c2*u(2);
               -c3*x(3)-b3*x(6)+c3*u(3)]; % Populate this with your dynamics.


%% Define a goal region.

%specify the terminal conditions
%setpoint = [-5;-2; 0];
%tspan = [0 60];

xT = [setpoint;zeros(3,1)];
u0 = setpoint;

%Have to change only this part to not depend on R
%its just goal region definition
Q = eye(n);
R = 10*eye(m);
[K0,S0,rho0] = ti_poly_lqr_roa(@(x,u) f0(0,x,u),xT,u0,Q,R);
S0 = 10*S0/rho0; %10

% Find the basin of attraction estimate 
% x0 = zeros(n,1);
% Q = eye(n);
% x = msspoly('x',n);
% 
% A = double(subs(diff(f0(0,x,zeros(3,1)),x),x,xT));
% 
% xbar = x;
% x = xbar + x0;
% P0 = lyap(A',Q);
% rho0 = ti_poly_roa(xbar,f0(0,x,u0),...
%                   xbar'*P0*xbar);
% P = P0/rho0;
% 
% % Calculate the contained ellispe.
% S0 = 1*inner_ellipse(P,x0,xT); %max 100

% xT is the center of the goal region.
% S0 defines the goal region (x-xT)'S(x-xT) <= 1.
% We use an estimate of the  basin of attraction for infinite
% horizon LQR, but choose your own.

%% Now the Timevarying case.

u0 = @(t) setpoint;

%integrating the system dynamics forward
[ts,xs] = ode45(@(t,x)f0(t,x,u0(t)),tspan,x0);

t0s = ts;
x0s = xs;

%check whether the final state lies within basin of attraction of setpoint
xf = x0s(end,:)';
if (xf-xT)'*S0*(xf-xT) > 1
    error('Final state not in the goal region. Increase tspan');
end

figure
%plot all the states
subplot(2,3,1);
plot(ts,xs(:,1));
ylabel('x');

subplot(2,3,2);
plot(ts,xs(:,2));
ylabel('y');

subplot(2,3,3);
plot(ts,xs(:,3));
ylabel('z');

subplot(2,3,4);
plot(ts,xs(:,4));
ylabel('v_x');

subplot(2,3,5);
plot(ts,xs(:,5));
ylabel('v_y');

subplot(2,3,6);
plot(ts,xs(:,6));
ylabel('v_z');

figure
plot(xs(:,1),xs(:,2));
xlabel('x');
ylabel('y');

