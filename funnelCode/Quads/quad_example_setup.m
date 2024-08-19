%% Disable Warning

warning('off','MATLAB:nearlySingularMatrix');

%% Define the Dynamics
n=6;   % Number of States
m=3; % Number of Inputs

% c/(s^2 + bs + c)

b_maxDisturbance = 0.05;
c_maxDisturbance = 0.05;

b1 = 1.85 + b_maxDisturbance*rand(); %2.05 %1.55
c1 = 0.56 + c_maxDisturbance*rand(); %0.96 %0.66

b2 = 1.65 + b_maxDisturbance*rand();
c2 = 0.62 + c_maxDisturbance*rand();

b3 = 0.2 + b_maxDisturbance*rand();
c3 = 0.0411 + c_maxDisturbance*rand();

% b1 = 0.3133;
% c1 = 0.0543;
% b2 = 0.3033;
% c2 = 0.0416;
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
%%  See how Ricatti works

xpp = spline(t0s,x0s'); %pp - piecewise polynomial

%spline computes the piecewise polynomial form of the 
%cubic spline interpolant to the x0 at the time stamps t0

%redefine dynamics, f to incorporate bounded disturbances as well
f0 = @(t,x,u) [    x(4); x(5); x(6); 
               -c1*x(1)-b1*x(4)+c1*u(1) + 0e-3; %+ abs(-c1*x(1)-b1*x(4))*0.1; 
               -c2*x(2)-b2*x(5)+c2*u(2) + 0e-3; %+ abs(-c2*x(2)-b2*x(5))*0.1;
               -c3*x(3)-b3*x(6)+c3*u(3) + 0e-3;]; %+ abs(-c3*x(3)-b3*x(6))*0.1]; % Populate this with your total dynamics.

           
[A,B] = tv_poly_linearize(f0,@(t) ppval(xpp,t),u0);

% You will need to replace these with appropriate cost functions for your
% LQR control task.
%Q = @(t) diag([5 5 5 1 1 1]);
Q = @(t) eye(n);


[ts,Ss] = tv_lyapunov(tspan,A,Q,S0); %solves the diff. lyapunov equation

Spp = spline(ts,Ss);
S = @(t) ppval(Spp,t);
taus = ts;

%check the sattelite example to turn on feedback control (Ss to Ps) 
%[ts,Ss] = tv_lqr_riccati(tspan,...
%                              A,B,...
%                              Q,R,S0);

%K = @(t) inv(R(t))*B(t)'*S(t);
%Ac = @(t) A(t) - B(t)*K(t);
%Q0  = @(t) (Q(t) + S(t)*B(t)*inv(R(t))*B(t)'*S(t));

%[taus,Ps] = tv_lyapunov(tspan,@(t) Ac(t),Q0,S0);
N = length(taus);
Ppp = interp1(taus,reshape(permute(Ss,[3 1 2]),N,n*n),'linear', ...
              'pp'); %Resample data at finer level
          
%% This example is undriven, so turn off feedback and nominal command.
input = setpoint.*ones(3,N);
upp = spline(taus,input);
K = @(t) zeros(m,n);