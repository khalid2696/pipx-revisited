function plot_swing_trajectory(funnel, outfile, opts)
%PLOT_SWING_TRAJECTORY Cart-pole swing-up or swing-down trajectory in the
%   (p, theta) plane, exported at a fixed size. By default only the nominal
%   trajectory is drawn (circle = initial state, square = terminal state).
%   ShowFunnel=true adds the funnel (envelope, cross-sections, terminal
%   outline) in the library-panel style, for structs whose certificates
%   actually define one.
%
%   plot_swing_trajectory(fUp,   './paper_figures/lib_cartpole_swingup',   ...
%       Width=w, Height=h, Label='Swing-up',   XLim=XL, YLim=YL);
%   plot_swing_trajectory(fDown, './paper_figures/lib_cartpole_swingdown', ...
%       Width=w, Height=h, Label='Swing-down', XLim=XL, YLim=YL);
%
%   funnel fields used: trajectory (4 x N), invarianceCertificates (4 x 4 x N),
%   with the funnel {x : (x - x_nom)' P (x - x_nom) <= 1}.
%   Pass the same XLim / YLim to both calls so the two boxes share axes.

arguments
    funnel
    outfile (1,:) char
    opts.Width        (1,1) double = 1.6     % total image width (in)
    opts.Height       (1,1) double = 2.4     % total image height (in)
    opts.Dims         (1,2) double = [1 3]   % state indices of p and theta
    opts.Label        (1,:) char   = ''      % e.g. 'Swing-up', drawn inside the box
    opts.XLim         (1,:) double = []      % [] = fit to the funnel
    opts.YLim         (1,:) double = []
    opts.EllipseEvery (1,1) double = 5       % cross-section every n-th knot
    opts.ShowFunnel   (1,1) logical = false  % true only if the certificates are a funnel
    opts.DPI          (1,1) double = 1000
end

d  = opts.Dims;
X  = funnel.trajectory;
N  = size(X, 2);
th = linspace(0, 2*pi, 64);  U = [cos(th); sin(th)];

if opts.ShowFunnel
    P = funnel.invarianceCertificates;
    % Exact shadow of each ellipsoid on (p, theta): boundary = c + Sig_dd^(1/2) U
    E = cell(1, N);
    for k = 1:N
        Sig  = inv((P(:, :, k) + P(:, :, k)') / 2);
        E{k} = X(d, k) + sqrtm(Sig(d, d)) * U;
    end
    % Swept envelope: union of convex hulls of consecutive cross-sections
    ws = warning('off', 'MATLAB:polyshape:repairedBySimplify');
    segs = repmat(polyshape, 1, max(N - 1, 1));
    for k = 1:N - 1
        pts = [E{k}, E{k + 1}];
        h   = convhull(pts(1, :)', pts(2, :)');
        segs(k) = polyshape(pts(1, h), pts(2, h));
    end
    env = union(segs);
    warning(ws);
    allPts = [E{:}];
else
    allPts = X(d, :);
end

% Limits
pad = 0.05 * (max(allPts, [], 2) - min(allPts, [], 2));
XL = opts.XLim;  if isempty(XL), XL = [min(allPts(1, :)) - pad(1), max(allPts(1, :)) + pad(1)]; end
YL = opts.YLim;  if isempty(YL), YL = [min(allPts(2, :)) - pad(2), max(allPts(2, :)) + pad(2)]; end

% Style (matches the library panels)
C = ijrr_colors();
cEnv   = 0.80 + 0.20 * C.branch;
cSlice = 0.40 + 0.60 * C.branch;
cDark  = C.branchTraj;

% Fixed geometry (same margins as plot_funnel_panel)
mL = 0.35;  mR = 0.2;  mB = 0.35;  mT = 0.1;
W = opts.Width;  H = opts.Height;
fig = ijrr_fig('single');
set(fig, 'Position', [1 1 W H], 'PaperSize', [W H], 'Resize', 'off', ...
    'PaperPositionMode', 'manual', 'PaperPosition', [0 0 W H]);
ax = axes(fig, 'Units', 'inches', 'Position', [mL mB W-mL-mR H-mB-mT], ...
    'PositionConstraint', 'innerposition');
set(groot, 'CurrentFigure', fig);  set(fig, 'CurrentAxes', ax);
hold(ax, 'on');
ax.Box = 'on';  ax.Layer = 'top';  ax.TickDir = 'in';

% Layers: [envelope -> slices -> terminal outline] -> trajectory -> markers
if opts.ShowFunnel
    plot(ax, env, 'FaceColor', cEnv, 'FaceAlpha', 1, 'EdgeColor', 'none');
    for k = 1 + opts.EllipseEvery : opts.EllipseEvery : N - 1
        plot(ax, E{k}(1, :), E{k}(2, :), '-', 'Color', cSlice, 'LineWidth', 0.35);
    end
    plot(ax, E{N}(1, :), E{N}(2, :), '-', 'Color', cDark, 'LineWidth', 0.6);
    lwTraj = 0.35;
else
    lwTraj = 0.9;                                                        % trajectory carries the panel
end
plot(ax, X(d(1), :), X(d(2), :), '-', 'Color', cDark, 'LineWidth', lwTraj);
plot(ax, X(d(1), 1), X(d(2), 1), 'o', 'MarkerSize', 4, 'LineWidth', 0.7, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');                     % initial state
plot(ax, X(d(1), N), X(d(2), N), 's', 'MarkerSize', 3.5, 'LineWidth', 0.5, ...
    'MarkerFaceColor', cDark, 'MarkerEdgeColor', 'k');                   % terminal state

% Axes: theta ticks at integer multiples of pi only
xlim(ax, XL);  ylim(ax, YL);
step = max(0.5, ceil(diff(XL) / 3 * 2) / 2);                             % about 3 x ticks
%ax.XTick = ceil(XL(1) / step) * step : step : XL(2);
ax.XTick = [-0.5 0 0.5];
kk = ceil(YL(1) / pi) : floor(YL(2) / pi);
ax.YTick = kk * pi;
lbl = cell(size(kk));
for i = 1:numel(kk)
    switch kk(i)
        case 0,  lbl{i} = '0';
        case 1,  lbl{i} = '\pi';
        case -1, lbl{i} = '-\pi';
        otherwise, lbl{i} = sprintf('%d\\pi', kk(i));
    end
end
ax.YTickLabel = lbl;
xlabel(ax, '{\itp} [m]');  ylabel(ax, '\theta [rad]');

if ~isempty(opts.Label)
    text(ax, 0.04, 0.97, opts.Label, 'Units', 'normalized', ...
        'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
        'BackgroundColor', 'w', 'Margin', 0.5);
end

% Export at fixed size (no cropping)
print(fig, outfile, '-dpng', sprintf('-r%d', opts.DPI));
end
