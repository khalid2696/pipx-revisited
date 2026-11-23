clc; clearvars; close all;

load("../precomputedFunnelLibrary/library.mat");
addpath("./plottingFns/");

% cartpoleLibrary = {};
% for x = -3:1:3
%     if x == 0
%         continue
%     end
% 
%     for level = -1:2:1
%         cartpoleLibrary{end+1} = funnelLibrary(num2str([x level]));
%     end
% end
% 
% plot_funnel_panel(cartpoleLibrary,  'cartpole',  '../paper_figures/cartpole_library', ...
%                     Dims=[1 3], EllipseEvery=3, DPI=1000);

%swing-trajectories
swingUpTrajectory = funnelLibrary(num2str([0 1])); %swing-up
swingDownTrajectory = funnelLibrary(num2str([0 -1])); %swing-down


XL = [-0.8, 0.8];  YL = [-0.95*pi, pi + 0.8];             % shared limits: set to cover both funnels
plot_swing_trajectory(swingUpTrajectory,   '../paper_figures/cartpole_library_swingup', ...
                            Width=1.5, Height=2.2, Label='Swing-up',   XLim=XL, YLim=YL);
plot_swing_trajectory(swingDownTrajectory, '../paper_figures/cartpole_library_swingdown', ...
                            Width=1.5, Height=2.2, Label='Swing-down', XLim=XL, YLim=YL);