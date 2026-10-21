clc; clearvars; close all;

load("../precomputedFunnelLibrary/library.mat");
addpath("./plottingFns/");

quadrotorLibrary = {};

for x = -4:1:4
    for y = -4:1:4
        if norm([x y], "inf") == 4 %just draw the borders
            quadrotorLibrary{end+1} = funnelLibrary(num2str([x y]));
        end
    end
end

plot_funnel_panel(quadrotorLibrary,  'quadrotor',  '../paper_figures/quadrotor_library', ...
                    Dims=[1 2], EllipseEvery=1, DPI=1000);