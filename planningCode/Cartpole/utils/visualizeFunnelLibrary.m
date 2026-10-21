clc; clearvars; close all;

load("../precomputedFunnelLibrary/library.mat");
addpath("./plottingFns/");

cartpoleLibrary = {};

for x = -3:1:3
    if x == 0
        continue
    end

    for level = -1:2:1
        cartpoleLibrary{end+1} = funnelLibrary(num2str([x level]));
    end
end

plot_funnel_panel(cartpoleLibrary,  'cartpole',  '../paper_figures/cartpole_library', ...
                    Dims=[1 3], EllipseEvery=3, DPI=1000);