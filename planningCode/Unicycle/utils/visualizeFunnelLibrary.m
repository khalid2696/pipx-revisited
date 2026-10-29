clc; clearvars; close all;

load("../precomputedFunnelLibrary/library.mat");
addpath("./plottingFns/");

unicycleLibrary = {};

for x = -2:1:2
    for y = -4:8:4
        unicycleLibrary{end+1} = funnelLibrary(num2str([x y]));
    end
end

return
% for x = -1:1:1
%     for y = -2:4:2
%         unicycleLibrary{end+1} = funnelLibrary(num2str([x y]));
%     end
% end

plot_funnel_panel(unicycleLibrary,  'unicycle',  '../paper_figures/unicycle_library', ...
                    Dims=[1 2], EllipseEvery=1, DPI=1000);