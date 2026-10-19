%% Example: two stacked panels with a shared time axis, single column
t = linspace(0, 10, 500);

fig = ijrr_fig('single', 0.75);
tl  = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile(tl);
plot(ax1, t, sin(t)); hold(ax1, 'on');
plot(ax1, t, cos(t), '--');                 % vary style, not only color
ylabel(ax1, 'State');
legend(ax1, {'x (m)', '\theta (rad)'}, 'Location', 'northeast');
xticklabels(ax1, {});                       % shared axis: label once

ax2 = nexttile(tl);
plot(ax2, t, exp(-0.3 * t));
xlabel(ax2, 'Time (s)');
%xlabel(ax, 'Time (s)', 'FontWeight', 'bold');
ylabel(ax2, '||e||_2');

linkaxes([ax1 ax2], 'x');
xlim(ax1, [0 10]);

ijrr_save(fig, 'fig_example', 'eps', Folder='figures');      % -> fig_example.eps (default)                                    
ijrr_save(fig, 'fig_example', 'png', Folder='figures');      % -> fig_example.eps (default)
                                    %use arg 'png' to save as png
