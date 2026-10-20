function [fig, ax] = sliverFig(robotPose, opts)
%SLIVERFIG One zoomed corridor snapshot, sized so N of them fill one column.
%   [fig, ax] = sliverFig(robotPose)                     % 6 per column
%   [fig, ax] = sliverFig(robotPose, NPerColumn=5, Ahead=20, Behind=5)
%
%   Call BEFORE the draw functions: limits are fixed here, so the drawing
%   cannot change them. Save with print (not exportgraphics) so every
%   sliver keeps exactly the same width:
%       print(fig, 'paper_figures/corridor_t1', '-dpng', '-r1000');

arguments
    robotPose (:,1) double
    opts.NPerColumn (1,1) double = 6
    opts.Corridor   (1,2) double = [-2.5 2.5]    % across the corridor (m)
    opts.YBounds    (1,2) double = [0 80]    % full corridor length (m)
    opts.Ahead      (1,1) double = 15        % shown toward the goal (m)
    opts.Behind     (1,1) double = 5         % shown behind the robot (m)
    opts.TickStep   (1,1) double = 5
end

COL_W   = 3.35;    % one column (in) -- set to \columnwidth / 72.27
GAP     = 0.05;    % horizontal gap between slivers in LaTeX (in)
LABEL_W = 0.20;    % reserved for y tick labels (in), identical on every sliver
PAD     = 0.04;    % top/bottom margin (in)

% Window along the corridor, kept inside the corridor bounds
yl = robotPose(2) + [-opts.Behind, opts.Ahead];
if yl(1) < opts.YBounds(1), yl = yl + (opts.YBounds(1) - yl(1)); end
if yl(2) > opts.YBounds(2), yl = yl - (yl(2) - opts.YBounds(2)); end

% Physical sizes: equal axes by construction
w   = COL_W / opts.NPerColumn - GAP;
axW = w - LABEL_W - PAD;
axH = axW * diff(yl) / diff(opts.Corridor);
h   = axH + 2*PAD;

fig = ijrr_fig('single');                       % template font/line defaults
set(fig, 'Position', [1 1 w h], 'PaperSize', [w h], ...
    'PaperPositionMode', 'manual', 'PaperPosition', [0 0 w h]);

ax = axes(fig, 'Units', 'inches', 'Position', [LABEL_W PAD axW axH]);
hold(ax, 'on');
xlim(ax, opts.Corridor);
ylim(ax, yl);
ax.DataAspectRatio = [1 1 1];
ax.XTick   = [];
ax.YTick   = ceil(yl(1)/opts.TickStep)*opts.TickStep : opts.TickStep : yl(2);
ax.Box     = 'on';
ax.Layer   = 'top';
ax.TickDir = 'in';
end
