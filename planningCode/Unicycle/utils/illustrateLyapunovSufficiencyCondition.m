%% Funnel invariance illustration: sufficient, but not necessary (unicycle)
%  1x3 panel, x-y-time view of one funnel with closed-loop rollouts that
%  start at random times along the horizon:
%    (a) starts inside the funnel        -> stays inside (certificate guarantee)
%    (b) starts outside, still converges -> condition is not necessary
%    (c) starts outside, misses          -> no guarantee outside the funnel
%  Late starts have little time left to recover, so most (c) cases come
%  from the second half of the horizon.
%
%  Assumptions (confirmed):
%    dynamics : xdot = v cos(th), ydot = v sin(th), thdot = w,  u = [v; w]
%    control  : u = u_nom - K (x - x_nom)
%    funnel   : (x - x_nom)' P (x - x_nom) <= 1
%    terminal : last funnel cross-section
%    x_nom, u_nom, K, P interpolated linearly between knots
%  Requires ijrr_fig.m and ijrr_colors.m on the path.

clear; close all;
addpath("./plottingFns/");
S  = load('funnel.mat'); F = S.funnel; %load sample funnel

%% ======================== User settings ================================
OUTFILE      = '../paper_figures/funnel_invariance';
N_SHOW       = 3;            % rollouts per panel
N_CAND       = 150;          % candidate (start time, initial state) pairs
T_START      = [0 0.4];     % start-time window, as a fraction of the horizon
S_IN         = [0.4 0.9];    % start scale inside:  V(x0) = s^2 < 1
S_OUT        = [1.3 4.0];    % start scale outside: V(x0) = s^2 > 1
P_INSIDE     = 0.3;          % fraction of candidates sampled inside
VIEW_AZEL    = [25 -20];      % camera azimuth / elevation (deg); try 25-50 / 15-30
BOX_ASPECT   = [1.2 1 1.1];  % plot-box x : y : time proportions
FIG_ASPECT   = 0.35;         % figure height / width (full text width)
SHOW_LEGEND  = true;         % shared legend above the panels
PANEL_LABELS = true;         % (a)/(b)/(c) inside panels; false if using subcaptions
rng(1);                      % reproducible sampling

%% ======================== Load funnel ==================================
t  = F.time_instances(:)';                 % 1 x N
X  = F.trajectory;                         % 3 x N
U  = F.nominalControl;                     % 2 x N
K  = F.feedbackControlGains;               % 2 x 3 x N
P  = F.invarianceCertificates;             % 3 x 3 x N
N  = numel(t);

Xt = X';                                   % N x 3, for interp1
Ut = U';                                   % N x 2
Kf = reshape(K, 6, N)';                    % N x 6
Pf = reshape(P, 9, N)';                    % N x 9
T  = t(end) - t(1);

%% ======================== Sample and classify rollouts =================
C    = ijrr_colors();
opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-9);

cand = struct('ts', {}, 'x0', {}, 'tq', {}, 'Y', {}, 'V', {}, ...
              'inside0', {}, 'success', {}, 'outsideXY', {});
for i = 1:N_CAND
    % Start time anywhere in the window, and the funnel slice at that time
    ts = t(1) + T * (T_START(1) + rand * diff(T_START));
    xn = interp1(t, Xt, ts, 'linear')';
    Ps = reshape(interp1(t, Pf, ts, 'linear'), 3, 3);  Ps = (Ps + Ps') / 2;

    % Initial state at a known funnel value V(x0, ts) = s^2
    if rand < P_INSIDE, s = S_IN(1)  + rand * diff(S_IN);
    else,               s = S_OUT(1) + rand * diff(S_OUT); end
    d  = randn(3, 1);  d = d / norm(d);
    x0 = xn + chol(Ps) \ (s * d);

    % Closed-loop rollout from ts to the end of the horizon
    tq = linspace(ts, t(end), 200);
    [~, Y] = ode45(@(tt, x) unicycleCL(tt, x, t, Xt, Ut, Kf), tq, x0, opts);
    V = arrayfun(@(j) funnelValue(tq(j), Y(j, :)', t, Xt, Pf), 1:numel(tq));

    e0xy = x0(1:2) - xn(1:2);
    cand(end + 1) = struct('ts', ts, 'x0', x0, 'tq', tq, 'Y', Y, 'V', V, ...
        'inside0',   s < 1, ...
        'success',   V(end) <= 1, ...
        'outsideXY', e0xy' * projectXY(Ps) * e0xy > 1);   % visibly outside the x-y slice
end

inS   = cand([cand.inside0]);
outOK = cand(~[cand.inside0] & [cand.success]  & [cand.outsideXY]);
outNG = cand(~[cand.inside0] & ~[cand.success] & [cand.outsideXY]);

% Sanity check: the certificate guarantees panel (a)
maxVin = max(arrayfun(@(c) max(c.V), inS));
if maxVin > 1 + 1e-3
    warning('funnel:violation', ['An inside start left the funnel (max V = %.3f). ' ...
        'Check the sign of K, the P normalization, or the angle wrapping.'], maxVin);
end
fprintf('Candidates: %d inside, %d outside->success, %d outside->failure\n', ...
    numel(inS), numel(outOK), numel(outNG));
if ~isempty(outNG)
    fprintf('Failures start at t = %.2f s (median), successes at t = %.2f s (median)\n', ...
        median([outNG.ts]), median([outOK.ts]));
end
if numel(outOK) < N_SHOW || numel(outNG) < N_SHOW
    warning('funnel:few', ['Too few outside starts in one class; ' ...
        'widen S_OUT, extend T_START toward 1, or raise N_CAND.']);
end

% (b): clearly outside at the start; (c): clearly outside at the end
V0 = arrayfun(@(c) c.V(1),   outOK);   [~, iB] = sort(V0, 'descend');
VT = arrayfun(@(c) c.V(end), outNG);   [~, iC] = sort(VT, 'descend');
sets = {pickSpread(inS, N_SHOW), ...
        pickSpread(outOK(iB(1:min(10, end))), N_SHOW), ...    % spread over time among the 10 farthest starts
        pickSpread(outNG(iC(1:min(10, end))), N_SHOW)};       % spread over time among the 10 worst misses

%% ======================== Common axis limits ===========================
th = linspace(0, 2*pi, 60);
lo = [inf; inf];  hi = -lo;
for k = 1:N
    E  = ellipseXY(X(1:2, k), projectXY(P(:,:,k)), th);
    lo = min(lo, min(E, [], 2));  hi = max(hi, max(E, [], 2));
end
for p = 1:3
    for c = sets{p}
        lo = min(lo, min(c.Y(:, 1:2), [], 1)');  hi = max(hi, max(c.Y(:, 1:2), [], 1)');
    end
end
pad = 0.05 * (hi - lo);
XL = [lo(1) - pad(1), hi(1) + pad(1)];
YL = [lo(2) - pad(2), hi(2) + pad(2)];

%% ======================== Plot =========================================
fig = ijrr_fig('double', FIG_ASPECT);
tl  = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

CLR.funnel   = [0.55 0.55 0.55];
CLR.nominal  = [0.15 0.15 0.15];
CLR.ok       = C.pipx;                 % blue: ends in terminal set
CLR.fail     = C.rrtx;                 % vermillion: misses terminal set
CLR.terminal = C.branch;               % green
labels = {'(a)', '(b)', '(c)'};

for p = 1:3
    ax = nexttile(tl);  hold(ax, 'on');

    % Funnel cross-sections (x-y shadow of each ellipsoid) stacked in time
    for k = 1:N
        E = ellipseXY(X(1:2, k), projectXY(P(:,:,k)), th);
        fill3(ax, E(1, :), E(2, :), t(k) * ones(1, numel(th)), CLR.funnel, ...
            'FaceAlpha', 0.18, 'EdgeColor', CLR.funnel, 'EdgeAlpha', 0.6, 'LineWidth', 0.3);
    end

    % Terminal set: last cross-section
    E = ellipseXY(X(1:2, N), projectXY(P(:,:,N)), th);
    fill3(ax, E(1, :), E(2, :), t(N) * ones(1, numel(th)), CLR.terminal, ...
        'FaceAlpha', 0.55, 'EdgeColor', CLR.terminal, 'LineWidth', 0.6);

    % Nominal trajectory
    plot3(ax, X(1, :), X(2, :), t, '--', 'Color', CLR.nominal, 'LineWidth', 0.9);

    % Closed-loop rollouts (each starts at its own time c.ts)
    for c = sets{p}
        if c.success, col = CLR.ok; else, col = CLR.fail; end
        plot3(ax, c.Y(:, 1), c.Y(:, 2), c.tq, '-', 'Color', col, 'LineWidth', 0.9);
        plot3(ax, c.Y(1, 1), c.Y(1, 2), c.tq(1), 'o', 'Color', col, ...
            'MarkerFaceColor', 'w', 'MarkerSize', 3.5, 'LineWidth', 0.8);       % initial
        if c.success
            plot3(ax, c.Y(end, 1), c.Y(end, 2), c.tq(end), 'o', 'Color', col, ...
                'MarkerFaceColor', col, 'MarkerSize', 2, 'LineWidth', 0.6);     % final, inside
        else
            plot3(ax, c.Y(end, 1), c.Y(end, 2), c.tq(end), 'x', 'Color', col, ...
                'MarkerSize', 3, 'LineWidth', 1.2);                             % final, outside
        end
    end

    % Axes: time increases downward, as in the reference figure
    xlim(ax, XL);  ylim(ax, YL);  zlim(ax, [t(1) t(end)]);
    ax.ZDir = 'reverse';
    view(ax, VIEW_AZEL(1), VIEW_AZEL(2));
    pbaspect(ax, BOX_ASPECT);
    grid(ax, 'on');  ax.GridAlpha = 0.10;  ax.Box = 'off';
    xlabel(ax, '{\itx} [m]');  ylabel(ax, '{\ity} [m]');  zlabel(ax, 'Time [s]');

    if PANEL_LABELS
        text(ax, 0.02, 0.98, labels{p}, 'Units', 'normalized', ...
            'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');
    end

    % Legend entries: invisible stand-ins in the first panel
    if p == 1 && SHOW_LEGEND
        h = gobjects(1, 6);
        h(1) = plot3(ax, nan, nan, nan, '--', 'Color', CLR.nominal, 'LineWidth', 0.9);
        h(2) = fill3(ax, nan, nan, nan, CLR.funnel, 'FaceAlpha', 0.3, 'EdgeColor', CLR.funnel);
        h(3) = fill3(ax, nan, nan, nan, CLR.terminal, 'FaceAlpha', 0.55, 'EdgeColor', CLR.terminal);
        h(4) = plot3(ax, nan, nan, nan, 'o', 'Color', [0.2 0.2 0.2], ...
            'MarkerFaceColor', 'w', 'MarkerSize', 3.5, 'LineWidth', 0.8);
        h(5) = plot3(ax, nan, nan, nan, '-', 'Color', CLR.ok,   'LineWidth', 0.9);
        h(6) = plot3(ax, nan, nan, nan, '-', 'Color', CLR.fail, 'LineWidth', 0.9);
        lgd = legend(ax, h, {'Nominal trajectory', 'Funnel', 'Terminal set', ...
            'Initial state', 'Reaches terminal set', 'Misses terminal set'}, ...
            'Orientation', 'horizontal', 'NumColumns', 3);
        lgd.Layout.Tile = 'north';
    end
end

%% ======================== Export (fixed size, no cropping) =============
print(fig, OUTFILE, '-dpng', '-r1000');

%% ======================== Local functions ==============================
function dx = unicycleCL(s, x, t, Xt, Ut, Kf)
% Closed-loop unicycle: u = u_nom - K (x - x_nom), linear interpolation in time
s  = min(max(s, t(1)), t(end));
xn = interp1(t, Xt, s, 'linear')';
un = interp1(t, Ut, s, 'linear')';
Kk = reshape(interp1(t, Kf, s, 'linear'), 2, 3);
e  = x - xn;  e(3) = wrapAngle(e(3));
u  = un - Kk * e;
dx = [u(1) * cos(x(3)); u(1) * sin(x(3)); u(2)];
end

function V = funnelValue(s, x, t, Xt, Pf)
% V(x, s) = e' P(s) e, with P interpolated linearly (stays positive definite)
s  = min(max(s, t(1)), t(end));
xn = interp1(t, Xt, s, 'linear')';
Pk = reshape(interp1(t, Pf, s, 'linear'), 3, 3);
e  = x - xn;  e(3) = wrapAngle(e(3));
V  = e' * Pk * e;
end

function a = wrapAngle(a)
a = atan2(sin(a), cos(a));
end

function P2 = projectXY(P)
% Exact shadow of {e : e' P e <= 1} on (x, y): all positions reachable at any heading
Sig = inv((P + P') / 2);
P2  = inv(Sig(1:2, 1:2));
end

function E = ellipseXY(c, P2, th)
[V, D] = eig((P2 + P2') / 2);
E = c + V * diag(1 ./ sqrt(diag(D))) * [cos(th); sin(th)];
end

function out = pickSpread(cands, n)
% Greedy farthest-point selection on start time, for rollouts spread over the horizon
n = min(n, numel(cands));
if n == 0, out = cands; return; end
ts = [cands.ts];
[~, first] = min(ts);  idx = first;              % earliest start first
for j = 2:n
    dmin = inf(1, numel(ts));
    for i = idx
        dmin = min(dmin, abs(ts - ts(i)));
    end
    dmin(idx) = -inf;
    [~, nxt] = max(dmin);
    idx(end + 1) = nxt; %#ok<AGROW>
end
out = cands(idx);
end