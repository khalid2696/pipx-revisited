%clc; clearvars; close all

%% checkFunnelPath_MCRollouts.m
%  Monte Carlo verification of a funnel-path (sequence of funnels) for a
%  12-state quadrotor under TVLQR feedback.
%
%  For each of num_samples rollouts we propagate the closed loop from the
%  start of the path to its end, injecting n state-perturbations *per funnel*
%  (so n*p total) at stratified-random knots. Each perturbation keeps the
%  state inside that knot's normalized invariant set { d : d'Pd <= 1 }.
%
%  We then verify, at every knot:
%    (1) funnel containment      : (x-x_nom)'P(x-x_nom) <= 1
%    (2) handoff inlet containment: carried state inside child funnel inlet
%    (3) collision                : x([1 2]) vs. 2D circular obstacles
%
%  Expected inputs (loaded/defined by a wrapper, or below):
%    funnelPath      : 1 x p struct array, IN TRAVERSAL ORDER, each with fields
%                        .time                       1 x N   (local, starts ~0)
%                        .trajectory_stateSpace      12 x N  (x_nom)
%                        .invariantSet_stateSpace    12x12xN (P_k, level <= 1)
%                        .feedforwardControlInputs   4 x N   (u_nom)
%                        .feedbackControlGains       4x12xN  (K_k)
%                        .trajectory_workSpace       2 x N   (optional, plotting)
%                        .invariantSet_workSpace     2x2xN   (optional, plotting)
%    quadParameters  : struct with .m, .g, .J   (REQUIRED, else error)
%    obstacles       : M x 3  ->  [x_c, y_c, r]  (optional; collision check)
%
%  If your funnels are stored keyed by .index with .parent/.child links,
%  see orderFunnelPath() at the bottom to build funnelPath first.

%% Add directories
addpath('../lib/');

%% ------------------------------------------------------------------------
%  Inputs / parameters (inherited from wrapper if present, else defaulted)
%  ------------------------------------------------------------------------
if ~exist('funnelPath','var')
    error('checkFunnelPath:noPath', ...
        'funnelPath (ordered struct array of funnels) is required.');
end
if ~exist('quadParameters','var')
    error('checkFunnelPath:noParams', ...
        'quadParameters (.m, .g, .J) is required.');
end

if ~exist('num_samples','var'),      num_samples = 200;         end % # rollouts
if ~exist('n_perturb','var'),        n_perturb   = 2;           end % perturbations PER funnel
if ~exist('perturbMode','var'),      perturbMode      = 'additive'; end % HOW  : 'additive'|'resample'
if ~exist('perturbPlacement','var'), perturbPlacement = 'boundary'; end % WHERE: 'boundary'|'interior'
if ~exist('perturbLevel','var'),     perturbLevel     = 1.0;    end % level c in (0,1], boundary=1
if ~exist('intMethod','var'),        intMethod   = 'RK4';       end % 'Euler'|'trapezoidal'|'RK4'|'ode45'
if ~exist('knotRange','var'),     knotRange   = [1 Inf];  end   % [kmin kmax] for perturb knots
if ~exist('robotRadius','var'),   robotRadius = 0.0;      end   % obstacle inflation (0 = none)
if ~exist('containTol','var'),    containTol  = 1e-6;     end   % slack on the <=1 test

hasObstacles = exist('obstacles','var') && ~isempty(obstacles);
if ~hasObstacles
    warning('checkFunnelPath:noObstacles', ...
        'No obstacles supplied -> collision check skipped.');
    obstacles = zeros(0,3);
end

dynamics = @(x,u) quadrotor_dynamics(x, u, quadParameters);

p       = numel(funnelPath);
n_x     = size(funnelPath(1).trajectory_stateSpace, 1);
n_u     = size(funnelPath(1).feedforwardControlInputs, 1);
ws_dims = [1 2];   % workspace (p_x, p_y) live in state indices 1,2

%% ------------------------------------------------------------------------
%  Build the global (concatenated) knot timeline ONCE.
%  Consecutive funnels share a boundary knot -> we keep funnel-1 knots
%  1..N and, for each later funnel, knots 2..N (drop the duplicate inlet).
%  ------------------------------------------------------------------------
[globalTime, knotMap, x_nom_global] = build_global_timeline(funnelPath, ws_dims);
T = numel(globalTime);   % total recorded knots along the path

fprintf('Funnel path: %d funnels, %d recorded knots, horizon %.3f s.\n', ...
    p, T, globalTime(end));

%% ------------------------------------------------------------------------
%  Monte Carlo rollouts
%  ------------------------------------------------------------------------
trajectories = cell(num_samples, 1);   % each 12 x T
input_traj   = cell(num_samples, 1);   % each  4 x T
funnelValue  = zeros(num_samples, T);  % (x-x_nom)'P(x-x_nom) at each knot
inFunnel     = false(num_samples, T);  % containment flag per knot
collided     = false(num_samples, 1);  % ever hit an obstacle
firstExitT   = nan(num_samples, 1);    % first funnel-exit time (NaN = never)
firstHitT    = nan(num_samples, 1);    % first collision time   (NaN = never)
handoffOK    = true(num_samples, max(p-1,1)); % child-inlet containment at each handoff

for s = 1:num_samples
    [X, U, V, hOK] = rollout_once(funnelPath, dynamics, ...
        n_perturb, perturbMode, perturbPlacement, perturbLevel, knotRange, intMethod);

    trajectories{s} = X;
    input_traj{s}   = U;
    funnelValue(s,:)= V;
    handoffOK(s,:)  = hOK;

    inside          = V <= 1 + containTol;
    inFunnel(s,:)   = inside;
    exitIdx         = find(~inside, 1, 'first');
    if ~isempty(exitIdx), firstExitT(s) = globalTime(exitIdx); end

    if hasObstacles
        [hit, hitIdx] = check_collisions(X(ws_dims,:), obstacles, robotRadius);
        collided(s)   = hit;
        if hit, firstHitT(s) = globalTime(hitIdx); end
    end
end

disp('-- End of Monte Carlo funnel-path rollouts --'); disp(' ');

%% ------------------------------------------------------------------------
%  Verification summary
%  ------------------------------------------------------------------------
nFullyContained = sum(all(inFunnel, 2));
knotOutRate     = 1 - mean(inFunnel(:));
nCollided       = sum(collided);
nHandoffFail    = sum(any(~handoffOK, 2));

fprintf('\n================ VERIFICATION SUMMARY ================\n');
fprintf('Rollouts                         : %d\n', num_samples);
fprintf('Perturbations per rollout        : %d  (%d/funnel x %d funnels)\n', ...
        n_perturb*p, n_perturb, p);
fprintf('Fully-contained rollouts         : %d / %d  (%.1f%%)\n', ...
        nFullyContained, num_samples, 100*nFullyContained/num_samples);
fprintf('Knot-instances outside funnel    : %.2f%%\n', 100*knotOutRate);
fprintf('Handoff-inlet violations         : %d rollout(s)\n', nHandoffFail);
fprintf('Max funnel value observed        : %.4f  (level = 1)\n', max(funnelValue(:)));
if hasObstacles
    fprintf('Rollouts colliding w/ obstacle   : %d / %d  (%.1f%%)\n', ...
            nCollided, num_samples, 100*nCollided/num_samples);
    if any(~isnan(firstHitT))
        fprintf('Earliest collision time          : %.3f s\n', min(firstHitT));
    end
end
if any(~isnan(firstExitT))
    fprintf('Earliest funnel-exit time        : %.3f s\n', min(firstExitT));
end
fprintf('======================================================\n\n');

%% ------------------------------------------------------------------------
%  Save trajectory data + normalized Lyapunov values (x'Px, boundary = 1)
%  ------------------------------------------------------------------------
if ~exist('saveResults','var'), saveResults = true; end
if ~exist('outputFile','var'),  outputFile  = 'funnelPath_MCRollouts.mat'; end

if saveResults
    rollout = struct();
    rollout.time       = globalTime;                 % 1 x T   (global knot times)
    rollout.states     = cat(3, trajectories{:});    % n_x x T x num_samples
    rollout.inputs     = cat(3, input_traj{:});      % n_u x T x num_samples
    rollout.lyapunov   = funnelValue;                % num_samples x T  (normalized: <=1 inside)
    rollout.inFunnel   = inFunnel;                   % num_samples x T  logical
    rollout.collided   = collided;                   % num_samples x 1
    rollout.firstExitT = firstExitT;                 % num_samples x 1
    rollout.firstHitT  = firstHitT;                  % num_samples x 1
    rollout.handoffOK  = handoffOK;                  % num_samples x (p-1)
    rollout.x_nom      = x_nom_global;               % n_x x T  nominal along path
    rollout.knotMap    = knotMap;                    % T x 2  [funnelIndex, localKnot]
    rollout.obstacles  = obstacles;                  % M x 3  [xc yc r]
    rollout.config     = struct( ...
        'num_samples',      num_samples, ...
        'n_perturb',        n_perturb, ...
        'perturbMode',      perturbMode, ...
        'perturbPlacement', perturbPlacement, ...
        'perturbLevel',     perturbLevel, ...
        'intMethod',        intMethod, ...
        'robotRadius',      robotRadius);

    save(outputFile, 'rollout', '-v7.3');
    fprintf('Saved trajectory data + normalized Lyapunov values -> %s\n\n', outputFile);
end

%% ------------------------------------------------------------------------
%  Visualization
%  ------------------------------------------------------------------------
disp('Plotting workspace, funnel value, and input profiles...'); disp(' ');

plot_workspace(trajectories, funnelPath, obstacles, robotRadius, ws_dims, collided);
plot_funnel_value(funnelValue, globalTime, inFunnel);
plot_input_profiles(input_traj, globalTime, funnelPath);

%% ========================================================================
%  Local functions
%  ========================================================================

function [X, U, V, handoffOK] = rollout_once(funnelPath, dynamics, ...
        n_perturb, mode, placement, level, knotRange, method)
    % One closed-loop rollout across the whole funnel path with n_perturb
    % stratified-random perturbations per funnel. Records at global knots
    % (funnel 1: knots 1..N; later funnels: knots 2..N to skip the shared
    % boundary instant already recorded by the parent).

    p   = numel(funnelPath);
    n_x = size(funnelPath(1).trajectory_stateSpace, 1);
    n_u = size(funnelPath(1).feedforwardControlInputs, 1);

    X = []; U = []; V = [];
    handoffOK = true(1, max(p-1,1));

    x = funnelPath(1).trajectory_stateSpace(:, 1);  % start on nominal inlet

    for fi = 1:p
        F     = funnelPath(fi);
        t     = F.time(:).';
        Nf    = numel(t);
        x_nom = F.trajectory_stateSpace;
        u_nom = F.feedforwardControlInputs;
        K     = F.feedbackControlGains;
        P     = F.invariantSet_stateSpace;

        % choose this funnel's perturbation knots (fresh every rollout)
        kmin = max(1, knotRange(1));
        kmax = min(Nf, knotRange(2));
        pk   = pick_perturbation_knots(kmin, kmax, n_perturb);

        for k = 1:Nf
            % --- funnel handoff: check child-inlet containment of carried state ---
            if fi > 1 && k == 1
                d = x - x_nom(:,1);
                if d.' * P(:,:,1) * d > 1 + 1e-6
                    handoffOK(fi-1) = false;
                end
            end

            % --- inject perturbation if this is a perturb knot ---
            if any(k == pk)
                x = perturb_within_ellipsoid(x, x_nom(:,k), P(:,:,k), mode, placement, level);
            end

            % --- closed-loop control (ZOH over the interval) ---
            u = u_nom(:,k) - K(:,:,k) * (x - x_nom(:,k));

            % --- record (skip duplicate shared boundary knot for fi>1) ---
            if ~(fi > 1 && k == 1)
                d          = x - x_nom(:,k);
                X(:,end+1) = x;               %#ok<AGROW>
                U(:,end+1) = u;               %#ok<AGROW>
                V(end+1)   = d.' * P(:,:,k) * d;  %#ok<AGROW>
            end

            % --- integrate to next knot within this funnel ---
            if k < Nf
                dt = t(k+1) - t(k);
                x  = integrate_step(dynamics, x, u, dt, method);
            end
        end
        % carry state across the handoff (state at funnel fi terminal knot)
    end
end

function pk = pick_perturbation_knots(kmin, kmax, n)
    % Stratified sampling: split [kmin,kmax] into n bins, one random knot each.
    % "As evenly spaced as possible" with per-bin jitter.
    if n <= 0, pk = []; return; end
    span = kmax - kmin + 1;
    n    = min(n, span);                       % can't place more than exist
    edges = round(linspace(kmin, kmax + 1, n + 1));
    pk = zeros(1, n);
    for b = 1:n
        lo = edges(b);
        hi = max(lo, edges(b+1) - 1);
        pk(b) = randi([lo, hi]);
    end
    pk = unique(pk);
end

function x_pert = perturb_within_ellipsoid(x_cur, x_nom, P, mode, placement, level)
    % Perturb so the result lies in the invariant set { d : d'Pd <= level }
    % (level <= 1), with the kick shaped by the ellipsoid so every state is
    % scaled correctly. Controlled by TWO independent flags:
    %
    %   mode      : how the deviation is formed
    %     'resample' - draw a FRESH deviation about the nominal (worst-case
    %                  funnel-invariance test, ignores the current state)
    %     'additive' - kick the CURRENT deviation (disturbance-on-trajectory)
    %
    %   placement : where the result lands within the set
    %     'boundary' - forced ONTO the level surface  (d'Pd = level)
    %     'interior' - anywhere inside, up to the level (d'Pd <= level)
    %
    % In every case the returned state satisfies d'Pd <= level.
    n  = numel(x_nom);
    A  = ellipsoid_map(P);                 % P^{-1/2}: unit ball -> invariant set
    yd = randn(n,1); yd = yd / norm(yd);   % random direction

    % target squared "radius" c for this draw:  d = A*sqrt(c)*yd  =>  d'Pd = c
    switch lower(placement)
        case 'boundary'
            c = level;                     % exactly on the level surface
        case 'interior'
            c = level * (rand^(1/n))^2;    % uniform-in-volume, up to the level
        otherwise
            error('perturb_within_ellipsoid:placement', ...
                  'Unknown placement "%s" (use boundary|interior).', placement);
    end

    switch lower(mode)
        case 'resample'
            d = A * (sqrt(c) * yd);        % fresh deviation about the nominal

        case 'additive'
            d = (x_cur - x_nom) + A * (sqrt(c) * yd);   % kick current deviation
            v = d.' * P * d;
            if strcmpi(placement, 'boundary')
                d = d * sqrt(level / v);   % scale onto the boundary (up or down)
            elseif v > level
                d = d * sqrt(level / v);   % keep interior: only project down if outside
            end

        otherwise
            error('perturb_within_ellipsoid:mode', ...
                  'Unknown mode "%s" (use additive|resample).', mode);
    end

    x_pert = x_nom + d;
end

function A = ellipsoid_map(P)
    % A = P^{-1/2} (symmetric). Maps the unit ball onto { d : d'Pd <= 1 }.
    P = (P + P.') / 2;
    [Vv, Dv] = eig(P);
    d = max(real(diag(Dv)), 1e-12);
    A = Vv * diag(1 ./ sqrt(d)) * Vv.';
end

function x_next = integrate_step(dynamics, x, u, dt, method)
    % One integration step holding u constant over [t, t+dt].
    switch lower(method)
        case 'euler'
            x_next = x + dt * dynamics(x, u);
        case 'trapezoidal'
            f1 = dynamics(x, u);
            f2 = dynamics(x + dt*f1, u);
            x_next = x + (dt/2)*(f1 + f2);
        case 'rk4'
            k1 = dynamics(x, u);
            k2 = dynamics(x + 0.5*dt*k1, u);
            k3 = dynamics(x + 0.5*dt*k2, u);
            k4 = dynamics(x + dt*k3, u);
            x_next = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
        otherwise  % ode45 (slower, accurate)
            [~, xo] = ode45(@(tt,xx) dynamics(xx, u), [0 dt], x);
            x_next = xo(end,:).';
    end
end

function [globalTime, knotMap, x_nom_global] = build_global_timeline(funnelPath, ws_dims)
    % Concatenate knots along the path, dropping shared boundary knots.
    % knotMap(g,:) = [funnelIndex, localKnot] for each global knot g.
    p = numel(funnelPath);
    globalTime = []; knotMap = []; x_nom_global = [];
    tOffset = 0;
    for fi = 1:p
        t  = funnelPath(fi).time(:).';
        Nf = numel(t);
        xn = funnelPath(fi).trajectory_stateSpace;
        if fi == 1, kStart = 1; else, kStart = 2; end   % skip duplicate inlet
        for k = kStart:Nf
            globalTime(end+1)   = tOffset + t(k);        %#ok<AGROW>
            knotMap(end+1,:)    = [fi, k];               %#ok<AGROW>
            x_nom_global(:,end+1) = xn(:,k);             %#ok<AGROW>
        end
        tOffset = tOffset + t(end);   % next funnel shares this instant
    end
end

function [hit, hitIdx] = check_collisions(xy, obstacles, robotRadius)
    % xy : 2 x T workspace path. obstacles : M x 3 [xc yc r]. No inflation if
    % robotRadius = 0. Returns first colliding knot index (T-based).
    hit = false; hitIdx = NaN;
    for t = 1:size(xy,2)
        pxy = xy(:,t);
        for o = 1:size(obstacles,1)
            c = obstacles(o,1:2).';
            r = obstacles(o,3) + robotRadius;
            if sum((pxy - c).^2) <= r^2
                hit = true; hitIdx = t; return;
            end
        end
    end
end

function plot_workspace(trajectories, funnelPath, obstacles, robotRadius, ws_dims, collided)
    figure; hold on; grid on; axis equal;

    % obstacles
    for o = 1:size(obstacles,1)
        draw_circle(obstacles(o,1), obstacles(o,2), obstacles(o,3), [0.85 0.33 0.33], 0.35);
        if robotRadius > 0
            draw_circle(obstacles(o,1), obstacles(o,2), obstacles(o,3)+robotRadius, ...
                        [0.85 0.33 0.33], 0.10);
        end
    end

    % funnel workspace cross-sections (every few knots to avoid clutter)
    for fi = 1:numel(funnelPath)
        if ~isfield(funnelPath(fi),'invariantSet_workSpace') || ...
           isempty(funnelPath(fi).invariantSet_workSpace), continue; end
        xw = funnelPath(fi).trajectory_workSpace;
        Mw = funnelPath(fi).invariantSet_workSpace;
        step = max(1, round(size(xw,2)/6));
        for k = 1:step:size(xw,2)
            eb = ellipsoid_map(Mw(:,:,k)) * ...
                 [cos(linspace(0,2*pi,60)); sin(linspace(0,2*pi,60))];
            plot(xw(1,k)+eb(1,:), xw(2,k)+eb(2,:), '-', ...
                 'Color', [0.6 0.6 0.6 0.5], 'LineWidth', 0.5);
        end
        plot(xw(1,:), xw(2,:), 'k--', 'LineWidth', 1.5);  % nominal workspace path
    end

    % rollouts (contained = blue, violating/colliding = red)
    for s = 1:numel(trajectories)
        X = trajectories{s};
        col = 'b'; if collided(s), col = 'r'; end
        plot(X(ws_dims(1),:), X(ws_dims(2),:), '-', 'Color', col, 'LineWidth', 0.4);
    end

    xlabel('p_x'); ylabel('p_y');
    title('MC Funnel-Path Rollouts (workspace)');
end

function draw_circle(xc, yc, r, faceColor, alpha)
    th = linspace(0, 2*pi, 100);
    fill(xc + r*cos(th), yc + r*sin(th), faceColor, ...
         'EdgeColor', faceColor*0.7, 'FaceAlpha', alpha);
end

function plot_funnel_value(funnelValue, time, inFunnel)
    figure; hold on; grid on;
    for s = 1:size(funnelValue,1)
        col = [0.5 0.5 0.9 0.4];
        if ~all(inFunnel(s,:)), col = [0.9 0.3 0.3 0.6]; end   % violating rollout
        plot(time, funnelValue(s,:), '-', 'Color', col, 'LineWidth', 0.4);
    end
    plot(time, mean(funnelValue,1), 'k-', 'LineWidth', 2);        % ensemble mean
    yline(1, 'r--', 'funnel level = 1', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('(x-x_{nom})^T P (x-x_{nom})');
    title('Funnel Containment Value Over Time');
    xlim([time(1) time(end)]);
end

function plot_input_profiles(input_traj, time, funnelPath)
    m = size(funnelPath(1).feedforwardControlInputs, 1);
    % concatenate nominal inputs along the path (match recorded knots)
    u_nom_g = []; 
    for fi = 1:numel(funnelPath)
        un = funnelPath(fi).feedforwardControlInputs;
        if fi == 1, kStart = 1; else, kStart = 2; end
        u_nom_g = [u_nom_g, un(:, kStart:end)]; %#ok<AGROW>
    end

    figure;
    labels = {'T','M_x','M_y','M_z'};
    for i = 1:m
        subplot(m,1,i); hold on; grid on;
        for s = 1:numel(input_traj)
            plot(time, input_traj{s}(i,:), 'b-', 'LineWidth', 0.3);
        end
        plot(time, u_nom_g(i,:), 'k--', 'LineWidth', 2);
        xlabel('time'); 
        if i <= numel(labels), ylabel(labels{i}); else, ylabel(sprintf('u_{%d}',i)); end
        if i == 1, title('Input Profiles from MC Rollouts'); end
    end
end

%% ------------------------------------------------------------------------
%  Optional: build an ordered funnelPath from a keyed set of funnels using
%  .index / .parent / .child links. Call BEFORE this script if needed:
%    funnelPath = orderFunnelPath(allFunnels, startIndex);
%  ------------------------------------------------------------------------
function funnelPath = orderFunnelPath(allFunnels, startIndex)
    % allFunnels : struct array with .index and .child (child index, [] = leaf)
    idx = [allFunnels.index];
    cur = startIndex; funnelPath = allFunnels([]);
    while ~isempty(cur) && ~(isscalar(cur) && (isnan(cur) || cur < 0))
        loc = find(idx == cur, 1);
        if isempty(loc), break; end
        funnelPath(end+1) = allFunnels(loc); %#ok<AGROW>
        cur = allFunnels(loc).child;
    end
end

%% ------------------------------------------------------------------------
%  Quadrotor dynamics (self-contained; remove if already on ../lib path)
%  State: x = [px;py;pz; vx;vy;vz; phi;theta;psi; p;q;r]   Input: u=[T;Mx;My;Mz]
%  ------------------------------------------------------------------------
function f = quadrotor_dynamics(x, u, quadParameters)
    m = quadParameters.m; g = quadParameters.g;
    Jxx = quadParameters.J(1); Jyy = quadParameters.J(2); Jzz = quadParameters.J(3);

    vx = x(4); vy = x(5); vz = x(6);
    phi = x(7); theta = x(8); psi = x(9);
    pp = x(10); qq = x(11); rr = x(12);

    T = u(1); Mp = u(2); Mq = u(3); Mr = u(4);

    c_phi = cos(phi); s_phi = sin(phi);
    c_theta = cos(theta); s_theta = sin(theta);
    c_psi = cos(psi); s_psi = sin(psi);
    t_theta = s_theta / c_theta;
    sec_theta = 1 / c_theta;

    f = [ vx;
          vy;
          vz;
          (T/m) * (c_phi * s_theta * c_psi + s_phi * s_psi);
          (T/m) * (c_phi * s_theta * s_psi - s_phi * c_psi);
          (T/m) * c_phi * c_theta - g;
          pp + qq * s_phi * t_theta + rr * c_phi * t_theta;
          qq * c_phi - rr * s_phi;
          qq * s_phi * sec_theta + rr * c_phi * sec_theta;
          (Mp + (Jyy - Jzz) * qq * rr) / Jxx;
          (Mq + (Jzz - Jxx) * pp * rr) / Jyy;
          (Mr + (Jxx - Jyy) * pp * qq) / Jzz ];
end
