%% Mohamed Khalid M Jaffar (July '24)

setupPlot();

load('funnelLibraryNominal.mat')
%sparse  - x and y spacing: 1    -- -5.0, -4.0 ... 3.0, 4.0, 5.0
%nominal - x and y spacing: 0.5  -- -5.0, -4.5 ... 4.0, 4.5, 5.0
%dense   - x and y spacing: 0.25 -- -5.0, -4.75 .. 4.25, 4.5, 4.75, 5.0

funnelList = values(funnelLibrary);
epsilon = 5;

%each funnel has attributes .time, .trajectory and .RofA
%to look up funnel from dictionary: funnelLibrary(num2str([delta_x, delta_y]))

for i=1:length(funnelList)
    thisFunnel = funnelList(i);

    if norm(thisFunnel.trajectory(1:2,1)) > epsilon %ignoring funnels that are more
        continue                                    %than an epsilon-distance away
    end

    plotFunnel(thisFunnel);
    drawnow
end

function setupPlot()
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
end

function plotFunnel(funnel)
    N = length(funnel.time);

    plot2DTrajectory(funnel.trajectory,funnel.time);
    
    %Plotting it backwards
    for i = N:-1:1
        %E = reshape(6,6,funnel.RofA(:,:,i));
    
        E = funnel.RofA(:,:,i);
        plotEllipse(E,funnel.trajectory(:,i),funnel.time(i));
        vol(i) = det(E^(-1/2));
    end
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

function plot2DTrajectory(x,t)

    plot3(x(1,:),x(2,:),t,'--k','LineWidth',2);
    %Plotting the end and start points
    plot3(x(1,1),x(2,1),t(1),'sg','LineWidth',3,'MarkerSize',7);
    plot3(x(1,end),x(2,end),t(end),'rx','LineWidth',3,'MarkerSize',7);

end