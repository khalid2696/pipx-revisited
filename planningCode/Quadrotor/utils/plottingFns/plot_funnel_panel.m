function plot_funnel_panel(lib, kind, outfile, opts)
%PLOT_FUNNEL_PANEL One panel of the funnel-library figure, exported at a fixed size.
%   plot_funnel_panel(uniLib,  'unicycle',  './paper_figures/lib_unicycle',  Dims=[1 2])
%   plot_funnel_panel(cartLib, 'cartpole',  './paper_figures/lib_cartpole',  Dims=[1 3])
%   plot_funnel_panel(quadLib, 'quadrotor', './paper_figures/lib_quadrotor', Dims=[1 2])    % top view
%   plot_funnel_panel(quadLib, 'quadrotor', './paper_figures/lib_quad3D',    Dims=[1 2 3])  % 3D view
%
%   lib: struct array or cell array of funnels with fields
%     trajectory             : n x N        full-state nominal trajectory
%     invarianceCertificates : n x n x N    ellipsoids {x : (x - c)' P (x - c) <= Level}
%   Dims selects the displayed state coordinates; each ellipsoid is projected
%   exactly (its shadow) onto them. Tubes are swept between consecutive knots.
%
%   All panels share width and left margin, so they align when stacked.
%   Width=2.2 for three panels side by side (figure*); default 3.35 for one column.
%   Requires ijrr_fig.m and ijrr_colors.m on the path.

arguments
    lib
    kind    (1,:) char {mustBeMember(kind, {'unicycle', 'cartpole', 'quadrotor'})}
    outfile (1,:) char
    opts.Dims      (1,:) double = [1 2]
    opts.TrajField (1,:) char   = 'trajectory'
    opts.CertField (1,:) char   = 'invarianceCertificates'
    opts.Level     (1,1) double = 1           % sublevel value: {x : x'Px <= Level}
    opts.LineStyle (1,:) char   = '-'        % nominal trajectory line style
    opts.Width     (1,1) double = 3.35        % total image width (in)
    opts.Height    (1,1) double = NaN         % plot-box height (in); NaN = default per kind
    opts.Every     (1,1) double = 1           % 3D only: knot stride for swept segments
    opts.EllipseEvery (1,1) double = 4        % 2D: draw a cross-section ellipse every n-th knot
    opts.DPI       (1,1) double = 1000
end

d    = opts.Dims;
is3D = numel(d) == 3;
if is3D && ~strcmp(kind, 'quadrotor')
    error('plot_funnel_panel:dims', '3D view is only defined for the quadrotor.');
end
if ~ismember(numel(d), [2 3])
    error('plot_funnel_panel:dims', 'Dims must have 2 or 3 entries.');
end

%% Extract funnels: centers and certificates scaled to level 1
L = asCell(lib);
F = cell(size(L));
for i = 1:numel(L)
    F{i}.c = L{i}.(opts.TrajField);
    F{i}.P = L{i}.(opts.CertField) / opts.Level;
end

%% Style
C = ijrr_colors();
S.fill = C.branch;  S.alpha = 0.22;  S.edge = C.branchTraj;  S.line = C.branchTraj;
S.envelope = 0.80 + 0.20 * C.branch;     % opaque light tint: overlaps do not darken
S.ellipse  = 0.40 + 0.60 * C.branch;     % intermediate cross-sections: lighter
S.ends     = C.branchTraj;               % inlet/outlet cross-sections: emphasized

%% Fixed geometry (identical left/right margins for every kind)
mL = 0.30;  mR = 0.06;  mB = 0.30;  mT = 0.05;
axW = opts.Width - mL - mR;
equalAxes = ~strcmp(kind, 'cartpole');

if is3D
    axH = 0.85 * axW;
else
    [xl, yl] = bounds2D(F, d, 0.05);
    if equalAxes
        axH = axW * diff(yl) / diff(xl);
    else
        axH = 0.32 * axW;                         % p-theta strip; units differ
    end
end
if ~isnan(opts.Height), axH = opts.Height; end
W = opts.Width;  H = mB + axH + mT;

fig = ijrr_fig('single');
set(fig, 'Position', [1 1 W H], 'PaperSize', [W H], 'Resize', 'off', ...
    'PaperPositionMode', 'manual', 'PaperPosition', [0 0 W H]);
ax = axes(fig, 'Units', 'inches', 'Position', [mL mB axW axH], ...
    'PositionConstraint', 'innerposition');
set(fig, 'CurrentAxes', ax);
hold(ax, 'on');
ax.Box = 'on';  ax.Layer = 'top';  ax.TickDir = 'in';

%% Draw
if ~is3D
    % Layered drawing so no funnel's envelope hides another's ellipses
    for k = 1:numel(F), drawEnvelope2D(ax, F{k}, d, S); end
    for k = 1:numel(F), drawEllipses2D(ax, F{k}, d, S, opts.EllipseEvery); end
    for k = 1:numel(F), plot(ax, F{k}.c(d(1), :), F{k}.c(d(2), :), opts.LineStyle, ...
            'Color', S.line, 'LineWidth', 0.25); end
    for k = 1:numel(F), drawMarkers2D(ax, F{k}, d, S); end
    xlim(ax, xl);  ylim(ax, yl);
    if equalAxes && isnan(opts.Height), daspect(ax, [1 1 1]); end
    if strcmp(kind, 'cartpole')
        setPiTicks(ax, yl);
        xlabel(ax, '{\itp} [m]');  ylabel(ax, '\theta [rad]');
    else
        xlabel(ax, '{\itx} [m]');  ylabel(ax, '{\ity} [m]');
    end
else
    for k = 1:numel(F), drawTube3D(ax, F{k}, d, S, opts.Every); end
    axis(ax, 'equal');  axis(ax, 'tight');
    zl = zlim(ax);  zFloor = zl(1) - 0.15 * diff(zl);
    for k = 1:numel(F), drawShadow(ax, F{k}, d(1:2), zFloor); end
    zlim(ax, [zFloor zl(2)]);
    view(ax, -35, 32);
    ax.Box = 'off';  grid(ax, 'on');  ax.GridAlpha = 0.08;
    camlight(ax, 'headlight');  lighting(ax, 'gouraud');
    xlabel(ax, '{\itx} (m)');  ylabel(ax, '{\ity} (m)');  zlabel(ax, '{\itz} (m)');
end

%% Export at fixed size (no cropping)
print(fig, outfile, '-dpng', sprintf('-r%d', opts.DPI));
end

%% ===================== Local functions ==================================
function L = asCell(lib)
if iscell(lib), L = lib(:)'; else, L = num2cell(lib(:)'); end
end

function Pp = project(P, d)
% Exact shadow of {x : x'Px <= 1} on coordinates d
Sig = inv(P);
Pp  = inv(Sig(d, d));
end

function X = ellipsoidPoints(c, P, U)
[V, D] = eig((P + P') / 2);
X = c + V * diag(1 ./ sqrt(diag(D))) * U;
end

function [xl, yl] = bounds2D(F, d, pad)
th = linspace(0, 2*pi, 32);  U = [cos(th); sin(th)];
lo = [inf; inf];  hi = -lo;
for i = 1:numel(F)
    for k = 1:size(F{i}.c, 2)
        E  = ellipsoidPoints(F{i}.c(d, k), project(F{i}.P(:, :, k), d), U);
        lo = min(lo, min(E, [], 2));  hi = max(hi, max(E, [], 2));
    end
end
r  = hi - lo;
xl = [lo(1) - pad*r(1), hi(1) + pad*r(1)];
yl = [lo(2) - pad*r(2), hi(2) + pad*r(2)];
end

function T = tubeUnion(f, d)
% Swept 2D tube: union of convex hulls of consecutive projected ellipses
th = linspace(0, 2*pi, 48);  U = [cos(th); sin(th)];
N  = size(f.c, 2);
E  = cell(1, N);
for k = 1:N
    E{k} = ellipsoidPoints(f.c(d, k), project(f.P(:, :, k), d), U);
end
ws = warning('off', 'MATLAB:polyshape:repairedBySimplify');
if N == 1
    T = polyshape(E{1}(1, :), E{1}(2, :));
else
    segs = repmat(polyshape, 1, N - 1);
    for k = 1:N - 1
        pts = [E{k}, E{k + 1}];
        h   = convhull(pts(1, :)', pts(2, :)');
        segs(k) = polyshape(pts(1, h), pts(2, h));
    end
    T = union(segs);
end
warning(ws);
end

function drawEnvelope2D(ax, f, d, S)
% Faint swept envelope: shows continuity between the drawn cross-sections
plot(ax, tubeUnion(f, d), 'FaceColor', S.envelope, 'FaceAlpha', 1, 'EdgeColor', 'none');
end

function drawEllipses2D(ax, f, d, S, every)
% Projected cross-section ellipses at selected knots (always first and last)
th = linspace(0, 2*pi, 64);  U = [cos(th); sin(th)];
N  = size(f.c, 2);
idx = unique([1:every:N, N]);
idx(idx == 1) = [];                      % no inlet outline: the fill already shows it
for k = idx
    E = ellipsoidPoints(f.c(d, k), project(f.P(:, :, k), d), U);
    if k == N
        plot(ax, E(1, :), E(2, :), '-', 'Color', S.ends, 'LineWidth', 0.6);     % outlet
    else
        plot(ax, E(1, :), E(2, :), '-', 'Color', S.ellipse, 'LineWidth', 0.35);
    end
end
end

function drawMarkers2D(ax, f, d, S)
c = f.c(d, :);
plot(ax, c(1, 1),   c(2, 1),   'o', 'MarkerSize', 4, 'LineWidth', 0.7, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');                 % initial state
plot(ax, c(1, end), c(2, end), 's', 'MarkerSize', 3.5, 'LineWidth', 0.5, ...
    'MarkerFaceColor', S.line, 'MarkerEdgeColor', 'k');              % terminal state
end

function drawTube3D(ax, f, d, S, every)
% Swept 3D tube: convex hulls of consecutive projected ellipsoids
c = f.c(d, :);
N = size(c, 2);
[sx, sy, sz] = sphere(12);
U = [sx(:) sy(:) sz(:)]';
idx = unique([1:every:N, N]);
X = cell(1, numel(idx));
for j = 1:numel(idx)
    X{j} = ellipsoidPoints(c(:, idx(j)), project(f.P(:, :, idx(j)), d), U);
end
for j = 1:numel(idx) - 1
    pts = [X{j}, X{j + 1}]';
    K = convhulln(pts);
    trisurf(K, pts(:, 1), pts(:, 2), pts(:, 3), 'Parent', ax, ...
        'FaceColor', S.fill, 'FaceAlpha', 0.12, 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud');
end
plot3(ax, c(1, :), c(2, :), c(3, :), '-', 'Color', S.line, 'LineWidth', 0.35);
plot3(ax, c(1, 1),   c(2, 1),   c(3, 1),   'o', 'MarkerSize', 2.5, 'LineWidth', 0.5, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', S.line);
plot3(ax, c(1, end), c(2, end), c(3, end), 'o', 'MarkerSize', 2.5, 'LineWidth', 0.5, ...
    'MarkerFaceColor', S.line, 'MarkerEdgeColor', S.line);
end

function drawShadow(ax, f, d2, z0)
% Filled light-gray footprint of the swept tube on the floor
T = tubeUnion(f, d2);
[xb, yb] = boundary(T);
breaks = [0; find(isnan(xb)); numel(xb) + 1];
for b = 1:numel(breaks) - 1
    r = breaks(b) + 1 : breaks(b + 1) - 1;
    if numel(r) > 2
        fill3(ax, xb(r), yb(r), z0 * ones(numel(r), 1), [0.88 0.88 0.88], ...
            'EdgeColor', 'none', 'FaceLighting', 'none');
    end
end
end

function setPiTicks(ax, yl)
k = ceil(yl(1) / (pi/2)) : floor(yl(2) / (pi/2));
ax.YTick = k * pi / 2;
ax.YTickLabel = arrayfun(@piLabel, k, 'UniformOutput', false);
end

function s = piLabel(k)
if k == 0, s = '0'; return; end
if mod(k, 2) == 0
    n = k / 2;
    if n == 1, s = '\pi'; elseif n == -1, s = '-\pi'; else, s = sprintf('%d\\pi', n); end
else
    if k == 1, s = '\pi/2'; elseif k == -1, s = '-\pi/2'; else, s = sprintf('%d\\pi/2', k); end
end
end