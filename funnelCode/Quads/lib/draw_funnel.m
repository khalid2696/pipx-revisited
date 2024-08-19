%close all

figure
view(3)
grid on
%axis equal
daspect([1 1 2]) %maintains aspect (scaling)
set(gca, 'ZDir','reverse') %invert the direction of z
hold on
xlabel('x');
ylabel('y');
zlabel('time');


%Plotting it backwards
for i = N-1:-1:1
    %Pi comes from Lagrange multiplier.m - substitute t in all polynomials with 0 
    St = double(reshape(subs(Pi(:,i),t,0),n,n)); %reshape into n-by-n matrix (n - dimension of system)
    
    %x0i comes from Lagrange multiplier.m - substitute t in all polynomials with 0
    xt = double(subs(x0i(:,i),t,0));
    %x = [setpoint', zeros(1,3)] - xt;
    
    %ppval evaluates piecewise polynomial at the given time
    %ell = (St./ppval(rhopp,ts(i)))^(-1/2)*[cos(th) ; sin(th)]; %find the ellipsoid at each timestamp
    %ell = Ss(:,:,N-i)^(-1/2)*[cos(th) ; sin(th)]; %Ellipse from the linearised
    %dynamics and riccati equation
    %fill(xt(1)+ell(1,:),xt(2)+ell(2,:),0.8*ones(1,3)) %offset the coordinates of the ellipsoid by value of 
                                                      %system states at that time step 
    M = St./ppval(rhopp,ts(i));
    %M = Ss(:,:,i);
    %plotEllipsoid(M,xt(1:3),ts(i));
    plotEllipse(M,xt(1:3),ts(i));
    %vol(i) = det(P);
    vol(i) = det(M^(-1/2));
end
%plotxy(x0s);
plot2DTrajectory(x0s,t0s); %Have to uncomment in the main code too
plotEllipse(S0,x0s(end,1:2),t0s(end));
%plotEllipsoid(S0,x0s(end,1:3),t0s(end));


%Function definitions
function plotxy(x)

    %figure
    axis equal
    grid on
    hold on
    xlabel('x');
    ylabel('y');
    plot(x(:,1),x(:,2),'--k','LineWidth',2);
    %Plotting the end and start points
    plot(x(1,1),x(1,2),'sg','LineWidth',3,'MarkerSize',7);
    plot(x(end,1),x(end,2),'rx','LineWidth',3,'MarkerSize',7);
    
    end
    
    function plot2DTrajectory(x,t)
    
    plot3(x(:,1),x(:,2),t,'--k','LineWidth',2);
    %Plotting the end and start points
    plot3(x(1,1),x(1,2),t(1),'sg','LineWidth',3,'MarkerSize',7);
    plot3(x(end,1),x(end,2),t(end),'rx','LineWidth',3,'MarkerSize',7);

end

function plot3DTrajectory(x)
    
    plot3(x(:,1),x(:,2),x(:,3),'--k','LineWidth',2);
    %Plotting the end and start points
    plot3(x(1,1),x(1,2),x(1,3),'sg','LineWidth',3,'MarkerSize',7);
    plot3(x(end,1),x(end,2),x(end,3),'rx','LineWidth',3,'MarkerSize',7);
    hold on
end

function plotEllipse(M,C,t)

    N = 50;
    
    th = linspace(-pi,pi,N);
    Basis = [1 0; 0 1; 0 0; 0 0; 0 0; 0 0]; %xy
    %Basis = [0 0; 0 1; 0 1; 0 0; 0 0; 0 0]; %yz
    %Basis = [1 0; 0 0; 0 1; 0 0; 0 0; 0 0];  %zx
    E = Basis'*inv(M)*Basis;
    
    ell = E^(1/2)*[cos(th); sin(th)];
    time = t*ones(length(ell),1);
    
    %fill(C(1) + ell(1,:),C(2) + ell(2,:),[0.8 0.8 0.8],'FaceAlpha',0.3);
    fill3(C(1) + ell(1,:),C(2) + ell(2,:),time,[0.8 0.8 0.8],'FaceAlpha',0.3);
end

function plotEllipsoid(M,C,t)

    N = 20; %no. of grid points
    
    Basis = [1 0 0; 0 1 0; 0 0 1; 0 0 0; 0 0 0; 0 0 0];
    E = Basis'*inv(M)*Basis;
    
    [~, D, V] = svd(E);
    %[V, D] = eig(E);
    
    a = sqrt(D(1,1));
    b = sqrt(D(2,2));
    c = sqrt(D(3,3));
    
    [X,Y,Z] = ellipsoid(0,0,0,a,b,c,N);
    
    XX = zeros(N+1,N+1);
    YY = zeros(N+1,N+1);
    ZZ = zeros(N+1,N+1);
    for i = 1:length(X)
        for j = 1:length(X)
            point = [X(i,j) Y(i,j) Z(i,j)]';
            P = V * point;
            XX(i,j) = P(1)+C(1);
            YY(i,j) = P(2)+C(2);
            ZZ(i,j) = P(3)+C(3);
        end
    end
    
    %can be used to have a nice gradient for time
    %tf = 60;
    %color = [0.7 1 0.5]*(t/tf);
    
    color = [0.8 0.8 0.8];
    %mesh(XX,YY,ZZ, 'FaceAlpha',0.1);
    %surf(XX,YY,ZZ,'FaceAlpha',0.5);
    surf(XX,YY,ZZ, 'EdgeColor', color, 'FaceColor', color,'FaceAlpha',0.1);
end