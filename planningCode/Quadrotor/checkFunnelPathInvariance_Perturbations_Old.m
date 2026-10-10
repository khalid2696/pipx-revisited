clc; clearvars; close all

%% checkFunnelPath_MCRollouts.m
%  Monte Carlo verification of a funnel-path (sequence of funnels) for a
%  12-state quadrotor under TVLQR feedback.
%
%  Each funnel is UPSAMPLED once onto a fine grid (spline for x_nom, linear
%  for u_nom/K/P) so the integrator sees a small step regardless of the coarse
%  knot spacing -- same scheme as the single-funnel rollout script. The upsampled
%  funnels are concatenated (shared boundary knot dropped) into one continuous
%  closed-loop path, with each funnel's own controller active over its segment.
%
%  For each of num_samples rollouts we propagate from the start of the path to
%  its end, injecting n state-perturbations *per funnel* (n*p total) at
%  stratified-random ORIGINAL knots (where P is certified). Each perturbation
%  keeps the state inside that knot's normalized invariant set { d : d'Pd <= 1 }.
%
%  We verify, on the fine grid:
%    (1) funnel containment       : (x-x_nom)'P(x-x_nom) <= 1
%    (2) handoff inlet containment : carried state inside child funnel inlet
%    (3) collision                 : x([1 2]) vs. 2D circular obstacles
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

%% Add directories
addpath('./lib/');

load('./temp_invariance_verification/sampleFunnelPath.mat'); %this variable needs to be named as funnelPath

% Quadrotor Parameters (imported from funnel computation code) [!!make sure both match!]
quadParameters.m = 0.7;                  % mass (kg)
quadParameters.g = 9.81;                 % gravity (m/s^2)
quadParameters.J = [2e-3, 2e-3, 3.5e-3]; % moment of inertia (kg⋅m^2)
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

if ~exist('num_samples','var'),      num_samples = 100;         end % # rollouts
if ~exist('n_perturb','var'),        n_perturb   = 5;           end % perturbations PER funnel
if ~exist('perturbMode','var'),      perturbMode      = 'additive'; end % HOW  : 'additive'|'resample'
if ~exist('perturbPlacement','var'), perturbPlacement = 'interior'; end % WHERE: 'boundary'|'interior'
if ~exist('perturbLevel','var'),     perturbLevel     = 0.02;    end % level c in (0,1], boundary=1
if ~exist('upsamplingFactor','var'), upsamplingFactor = 50;     end % fine steps per ORIGINAL knot interval
if ~exist('intMethod','var'),        intMethod   = 'RK4';       end % 'Euler'|'trapezoidal'|'RK4'|'ode45'
if ~exist('divergeTol','var'),       divergeTol  = 1e6;         end % |x| beyond this (or non-finite) => rollout flagged diverged
if ~exist('knotRange','var'),        knotRange   = [1 Inf];     end % [kmin kmax] for perturb knots (ORIGINAL knots)
if ~exist('robotRadius','var'),      robotRadius = 0.0;         end % obstacle inflation (0 = none)
if ~exist('containTol','var'),       containTol  = 1e-6;        end % slack on the <=1 test
if ~exist('angleIdx','var'),         angleIdx    = [7 8 9];     end % Euler-angle indices: deviations wrapped to [-pi,pi]

hasObstacles = exist('obstacles','var') && ~isempty(obstacles);
if ~hasObstacles
    warning('checkFunnelPath:noObstacles', ...
        'No obstacles supplied -> collision check skipped.');
    obstacles = zeros(0,3);
end

dynamics = @(x,u) quadrotor_dynamics(x, u, quadParameters);

p       = numel(funnelPath);
n_x     = size(funnelPath{1}.trajectory_stateSpace, 1);
n_u     = size(funnelPath{1}.feedforwardControlInputs, 1);
ws_dims = [1 2];   % workspace (p_x, p_y) live in state indices 1,2

%% ------------------------------------------------------------------------
%  Upsample + concatenate the funnel path ONCE (not per rollout).
%  upFunnels(fi) carries the fine-grid time/x_nom/u_nom/K/P and the fine
%  indices of the original knots. globalTime/x_nom_global/u_nom_global are the
%  concatenated fine path (shared boundary knot dropped).
%  ------------------------------------------------------------------------
[upFunnels, globalTime, knotMap, x_nom_global, u_nom_global] = ...
    preprocess_funnels(funnelPath, upsamplingFactor);
T = numel(globalTime);   % total recorded (fine) points along the path

fprintf('Funnel path: %d funnels, upsampling x%d -> %d fine points, horizon %.3f s.\n', ...
    p, upsamplingFactor, T, globalTime(end));

% ---- certified-knot columns (metrics/plots/saves restricted to these) ----
knotCols   = find(mod(knotMap(:,2) - 1, upsamplingFactor) == 0).';  % 1 x Tk, row
knotTime   = globalTime(knotCols);
x_nom_knot = x_nom_global(:, knotCols);
u_nom_knot = u_nom_global(:, knotCols);
knotMap_k  = knotMap(knotCols, :);
Tk         = numel(knotCols);
fprintf('Certified knots along path       : %d\n', Tk);

%% --- Test D: seam continuity between consecutive funnels ---
seamGap = zeros(1, p-1);
for fi = 1:p-1
    seamGap(fi) = norm(funnelPath{fi}.trajectory_stateSpace(:,end) ...
                     - funnelPath{fi+1}.trajectory_stateSpace(:,1));
end
fprintf('Seam gap  state: max %.3g, mean %.3g (over %d handoffs)\n', ...
        max(seamGap), mean(seamGap), p-1);

%% --- Test E: is the nominal (x_nom,u_nom) RK4-feasible under this f? ---
maxDefectV = 0;
for fi = 1:numel(upFunnels)
    Ff = upFunnels(fi); t = Ff.time; ki = Ff.knotFineIdx;
    for j = 1:numel(ki)-1
        a = ki(j); b = ki(j+1);
        x = Ff.x_nom(:,a);                          % start exactly on a certified knot
        for k = a:b-1                               % feedforward only, NO feedback
            x = integrate_step(dynamics, x, Ff.u_nom(:,k), t(k+1)-t(k), intMethod);
        end
        d = x - Ff.x_nom(:,b);
        d(angleIdx) = mod(d(angleIdx)+pi, 2*pi) - pi;
        maxDefectV = max(maxDefectV, d.' * Ff.P(:,:,b) * d);  % P-metric, comparable to V
    end
end
fprintf('Max feedforward defect (P-metric) : %.4g\n', maxDefectV);

%% --- Test F: trapezoidal-collocation residual of the planner nominal ---
maxTrapV = 0;
for fi = 1:numel(funnelPath)
    F = funnelPath{fi}; t = F.time; N = numel(t);
    xn = F.trajectory_stateSpace; un = F.feedforwardControlInputs;
    P  = F.invariantSet_stateSpace;
    for k = 1:N-1
        H  = t(k+1) - t(k);
        f1 = dynamics(xn(:,k),   un(:,k));
        f2 = dynamics(xn(:,k+1), un(:,k+1));
        d  = xn(:,k+1) - xn(:,k) - (H/2)*(f1 + f2);     % implicit trapezoid residual
        d(angleIdx) = mod(d(angleIdx)+pi, 2*pi) - pi;
        maxTrapV = max(maxTrapV, d.' * P(:,:,k+1) * d);
    end
end
fprintf('Max trapezoidal-collocation residual (P-metric) : %.4g\n', maxTrapV);

%% ------------------------------------------------------------------------
%  Monte Carlo rollouts
%  ------------------------------------------------------------------------
trajectories = cell(num_samples, 1);   % each n_x x T
input_traj   = cell(num_samples, 1);   % each n_u x T
funnelValue  = zeros(num_samples, T);  % (x-x_nom)'P(x-x_nom) on the fine grid
inFunnel     = false(num_samples, T);  % containment flag per fine point
collided     = false(num_samples, 1);  % ever hit an obstacle
firstExitT   = nan(num_samples, 1);    % first funnel-exit time (NaN = never)
firstHitT    = nan(num_samples, 1);    % first collision time   (NaN = never)
handoffOK    = true(num_samples, max(p-1,1)); % child-inlet containment at each handoff
diverged     = false(num_samples, 1);  % numerical blow-up flag

for s = 1:num_samples
    [X, U, V, hOK, dvg] = rollout_once(upFunnels, dynamics, ...
        n_perturb, perturbMode, perturbPlacement, perturbLevel, knotRange, ...
        intMethod, divergeTol, angleIdx);

    diverged(s)     = dvg;
    trajectories{s} = X;
    input_traj{s}   = U;
    funnelValue(s,:)= V;
    handoffOK(s,:)  = hOK;

    inside          = V <= 1 + containTol;
    inFunnel(s,:)   = inside;
    vK              = V(knotCols);
    exitIdxK        = find(vK > 1 + containTol, 1, 'first');   % true exit; NaN tail ignored
    if ~isempty(exitIdxK), firstExitT(s) = knotTime(exitIdxK); end

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
funnelValueK    = funnelValue(:, knotCols);
valid           = isfinite(funnelValueK);                 % S x Tk: knots actually reached
inFunnelK       = funnelValueK <= 1 + containTol;         % NaN -> false (handled via `valid`)
% exit rate over REACHED knots only (don't count NaN tails of diverged runs as exits)
knotOutRate     = 1 - sum(inFunnelK(:)) / max(nnz(valid), 1);
% "fully contained" = did not diverge AND never left a funnel
nFullyContained = sum( ~diverged & all(inFunnelK, 2) );
nCollided       = sum(collided);
nHandoffFail    = sum(any(~handoffOK, 2));

fprintf('\n================ VERIFICATION SUMMARY ================\n');
fprintf('Rollouts                         : %d\n', num_samples);
fprintf('Perturbations per rollout        : %d  (%d/funnel x %d funnels)\n', ...
        n_perturb*p, n_perturb, p);
fprintf('Fully-contained rollouts         : %d / %d  (%.1f%%)\n', ...
        nFullyContained, num_samples, 100*nFullyContained/num_samples);
fprintf('Knots outside funnel             : %.2f%%\n', 100*knotOutRate);
fprintf('Handoff-inlet violations         : %d rollout(s)\n', nHandoffFail);
if any(diverged)
    fprintf(2, 'Diverged (numerical blow-up)     : %d rollout(s)  <-- integration/control issue, not a funnel result\n', sum(diverged));
end
fprintf('Max funnel value observed        : %.4g  (level = 1)\n', max(funnelValueK(:), [], 'omitnan'));
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

fprintf('Max V at certified knots : %.4g   (vs %.4g on full fine grid)\n', ...
        max(funnelValueK(:), [], 'omitnan'), max(funnelValue(:), [], 'omitnan'));
%% ------------------------------------------------------------------------
%  Save trajectory data + normalized Lyapunov values (x'Px, boundary = 1)
%  ------------------------------------------------------------------------
if ~exist('saveResults','var'), saveResults = true; end
if ~exist('outputFile','var'),  outputFile  = 'funnelPath_MCRollouts.mat'; end

if saveResults
    rollout = struct();
    allStates          = cat(3, trajectories{:});   % n_x x T x S (fine, transient)
    allInputs          = cat(3, input_traj{:});
    rollout.time       = knotTime;                  % 1 x Tk
    rollout.states     = allStates(:, knotCols, :); % n_x x Tk x S
    rollout.inputs     = allInputs(:, knotCols, :); % n_u x Tk x S
    rollout.lyapunov   = funnelValue(:, knotCols);  % S x Tk  (certified P)
    rollout.inFunnel   = inFunnel(:, knotCols);     % S x Tk
    ...
    rollout.x_nom      = x_nom_knot;                % n_x x Tk
    rollout.u_nom      = u_nom_knot;                % n_u x Tk
    rollout.knotMap    = knotMap_k;                 % Tk x 2
    rollout.obstacles  = obstacles;                  % M x 3  [xc yc r]
    rollout.config     = struct( ...
        'num_samples',      num_samples, ...
        'n_perturb',        n_perturb, ...
        'perturbMode',      perturbMode, ...
        'perturbPlacement', perturbPlacement, ...
        'perturbLevel',     perturbLevel, ...
        'upsamplingFactor', upsamplingFactor, ...
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
plot_funnel_value(funnelValue(:,knotCols), knotTime, inFunnel(:,knotCols));
plot_input_profiles(input_traj, globalTime, u_nom_global);

%% ========================================================================
%  Local functions
%  ========================================================================

function [X, U, V, handoffOK, diverged] = rollout_once(upFunnels, dynamics, ...
        n_perturb, mode, placement, level, knotRange, method, divergeTol, angleIdx)
    % One closed-loop rollout across the upsampled funnel path. Within each
    % funnel we step the fine grid one RK4 step at a time, recomputing
    % u = u_nom - K(x-x_nom) from the fine (interpolated) references at every
    % point. Perturbations are injected only at ORIGINAL knots (fine indices in
    % .knotFineIdx), where P is certified. Records at global fine points
    % (funnel 1: all; later funnels: drop the shared inlet point).

    p   = numel(upFunnels);
    n_x = size(upFunnels(1).x_nom, 1);
    n_u = size(upFunnels(1).u_nom, 1);

    % preallocate to the global recorded length
    T = size(upFunnels(1).x_nom, 2);
    for fi = 2:p, T = T + size(upFunnels(fi).x_nom, 2) - 1; end
    X = nan(n_x, T); U = nan(n_u, T); V = nan(1, T);
    handoffOK = true(1, max(p-1,1));
    diverged  = false;
    col = 0;

    x = upFunnels(1).x_nom(:, 1);   % start on nominal inlet

    for fi = 1:p
        Ff      = upFunnels(fi);
        t       = Ff.time;
        Ndf     = numel(t);
        x_nom   = Ff.x_nom;
        u_nom   = Ff.u_nom;
        K       = Ff.K;
        P       = Ff.P;
        knotIdx = Ff.knotFineIdx;          % fine indices of original knots
        Nf_orig = numel(knotIdx);

        % choose this funnel's perturbation knots (ORIGINAL knots), then map to fine
        kmin   = max(1, knotRange(1));
        kmax   = min(Nf_orig, knotRange(2));
        pkOrig = pick_perturbation_knots(kmin, kmax, n_perturb);
        pkFine = knotIdx(pkOrig);

        for k = 1:Ndf
            % --- funnel handoff: child-inlet containment of carried state ---
            if fi > 1 && k == 1
                d = wrap_dev(x - x_nom(:,1), angleIdx);
                if d.' * P(:,:,1) * d > 1 + 1e-6
                    handoffOK(fi-1) = false;
                end
            end

            % --- inject perturbation at an original knot ---
            if any(k == pkFine)
                x = perturb_within_ellipsoid(x, x_nom(:,k), P(:,:,k), mode, placement, level, angleIdx);
            end

            % --- closed-loop control from fine references (wrapped deviation) ---
            u = u_nom(:,k) - K(:,:,k) * wrap_dev(x - x_nom(:,k), angleIdx);

            % --- record (skip duplicate shared boundary point for fi>1) ---
            if ~(fi > 1 && k == 1)
                col      = col + 1;
                d        = wrap_dev(x - x_nom(:,k), angleIdx);
                X(:,col) = x;
                U(:,col) = u;
                V(col)   = d.' * P(:,:,k) * d;
            end

            % --- one integration step over the (fine) interval, u held ---
            if k < Ndf
                dt = t(k+1) - t(k);
                x  = integrate_step(dynamics, x, u, dt, method);

                if any(~isfinite(x)) || norm(x) > divergeTol
                    diverged = true; return;   % remaining columns stay NaN
                end
            end
        end
        % carry state across the handoff to the next funnel's controller
    end
end

function [upFunnels, globalTime, knotMap, x_nom_global, u_nom_global] = ...
        preprocess_funnels(funnelPath, uf)
    % Upsample each funnel onto a fine grid (original knots land exactly on
    % fine indices 1:uf:end), then concatenate into one global fine path,
    % dropping the shared boundary point between consecutive funnels.
    p = numel(funnelPath);
    upFunnels = struct('time',{},'x_nom',{},'u_nom',{},'K',{},'P',{},'knotFineIdx',{});

    globalTime = []; knotMap = []; x_nom_global = []; u_nom_global = [];
    tOffset = 0;

    for fi = 1:p
        F  = funnelPath{fi};
        t  = F.time(:).';
        Nf = numel(t);

        % fine local time grid that includes every original knot exactly
        tf = t(1);
        for k = 1:Nf-1
            seg = linspace(t(k), t(k+1), uf+1);
            tf  = [tf, seg(2:end)];               %#ok<AGROW>
        end
        Ndf      = numel(tf);                      % 1 + (Nf-1)*uf
        knotFine = 1:uf:Ndf;                       % fine indices of original knots

        % interpolate: spline for state, linear for inputs/gains/sets
        xf = zeros(size(F.trajectory_stateSpace,1), Ndf);
        for i = 1:size(xf,1)
            xf(i,:) = interp1(t, F.trajectory_stateSpace(i,:), tf, 'spline');
        end
        uf_ = zeros(size(F.feedforwardControlInputs,1), Ndf);
        for i = 1:size(uf_,1)
            uf_(i,:) = interp1(t, F.feedforwardControlInputs(i,:), tf, 'linear');
        end
        Kf = upsample_matrix(F.feedbackControlGains,  t, tf);
        Pf = upsample_matrix(F.invariantSet_stateSpace, t, tf);

        upFunnels(fi).time        = tf;
        upFunnels(fi).x_nom       = xf;
        upFunnels(fi).u_nom       = uf_;
        upFunnels(fi).K           = Kf;
        upFunnels(fi).P           = Pf;
        upFunnels(fi).knotFineIdx = knotFine;

        % concatenate into the global fine path (drop shared inlet for fi>1)
        if fi == 1, kStart = 1; else, kStart = 2; end
        for k = kStart:Ndf
            globalTime(end+1)     = tOffset + tf(k);  %#ok<AGROW>
            knotMap(end+1,:)      = [fi, k];          %#ok<AGROW>
            x_nom_global(:,end+1) = xf(:,k);          %#ok<AGROW>
            u_nom_global(:,end+1) = uf_(:,k);         %#ok<AGROW>
        end
        tOffset = tOffset + tf(end);   % next funnel shares this instant
    end
end

function M_fine = upsample_matrix(M_coarse, t_coarse, t_fine)
    % Element-wise linear interpolation of a d1 x d2 x N matrix sequence.
    [d1, d2, ~] = size(M_coarse);
    Nf = numel(t_fine);
    M_fine = zeros(d1, d2, Nf);
    for i = 1:d1
        for j = 1:d2
            M_fine(i,j,:) = interp1(t_coarse, squeeze(M_coarse(i,j,:)), t_fine, 'linear');
        end
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

function x_pert = perturb_within_ellipsoid(x_cur, x_nom, P, mode, placement, level, angleIdx)
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
            d = wrap_dev(x_cur - x_nom, angleIdx) + A * (sqrt(c) * yd);   % kick current (wrapped) deviation
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

function dw = wrap_dev(d, angleIdx)
    % Wrap the Euler-angle components of a state deviation to [-pi, pi], so a
    % nominal angle near +-pi does not produce a spurious ~2*pi deviation.
    dw = d;
    dw(angleIdx) = mod(dw(angleIdx) + pi, 2*pi) - pi;
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

function [hit, hitIdx] = check_collisions(xy, obstacles, robotRadius)
    % xy : 2 x T workspace path. obstacles : M x 3 [xc yc r]. No inflation if
    % robotRadius = 0. Returns first colliding fine-point index.
    hit = false; hitIdx = NaN;
    for t = 1:size(xy,2)
        pxy = xy(:,t);
        if any(~isfinite(pxy)), continue; end
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
        if ~isfield(funnelPath{fi},'invariantSet_workSpace') || ...
           isempty(funnelPath{fi}.invariantSet_workSpace), continue; end
        xw = funnelPath{fi}.trajectory_workSpace;
        Mw = funnelPath{fi}.invariantSet_workSpace;
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
        plot(time, funnelValue(s,:), '.-', 'Color', col, 'LineWidth', 0.4, 'MarkerSize', 4);
    end
    plot(time, mean(funnelValue,1,'omitnan'), 'k.-', 'LineWidth', 2, 'MarkerSize', 8); % ensemble mean
    yline(1, 'r--', 'funnel level = 1', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('(x-x_{nom})^T P (x-x_{nom})');
    title('Funnel Containment Value Over Time');
    xlim([time(1) time(end)]);
end

function plot_input_profiles(input_traj, time, u_nom_global)
    m = size(u_nom_global, 1);
    figure;
    labels = {'T','M_x','M_y','M_z'};
    for i = 1:m
        subplot(m,1,i); hold on; grid on;
        for s = 1:numel(input_traj)
            plot(time, input_traj{s}(i,:), 'b-', 'LineWidth', 0.3);
        end
        plot(time, u_nom_global(i,:), 'k--', 'LineWidth', 2);
        xlabel('time');
        if i <= numel(labels), ylabel(labels{i}); else, ylabel(sprintf('u_{%d}',i)); end
        if i == 1, title('Input Profiles from MC Rollouts'); end
    end
end

%% ------------------------------------------------------------------------
% Quadrotor dynamics (self-contained) (imported from funnel computation code)
function f = quadrotor_dynamics(x, u, quadParameters)
    
    %extracting quadrotor model parameters
    m = quadParameters.m; g = quadParameters.g;
    Jxx = quadParameters.J(1); Jyy = quadParameters.J(2); Jzz = quadParameters.J(3);
    
    %assigning state variables for ease of usage
    % State: x = [px; py; pz; vx; vy; vz; phi; theta; psi; p; q; r]
    
    %px = x(1); py = x(2); pz = x(3);
    vx = x(4); vy = x(5); vz = x(6);
    phi = x(7); theta = x(8); psi = x(9);
    p = x(10); q = x(11); r = x(12);

    %assigning input variables for ease of usage
    % Input: u = [T; Mx; My; Mz]
    T = u(1); Mp = u(2); Mq = u(3); Mr = u(4);
    
    % Trigonometric shortcuts
    c_phi = cos(phi); s_phi = sin(phi);
    c_theta = cos(theta); s_theta = sin(theta);
    c_psi = cos(psi); s_psi = sin(psi);
    t_theta = s_theta/c_theta;
    sec_theta = 1/c_theta;
    
    % Quadrotor dynamics 
    % Assumptions: no aerodynamic drag and gyroscopic coupling due to rotor inertia)
    f = [
        % Position derivatives
        vx;
        vy;
        vz;
        
        % Velocity derivatives
        (T/m) * (c_phi * s_theta * c_psi + s_phi * s_psi);
        (T/m) * (c_phi * s_theta * s_psi - s_phi * c_psi);
        (T/m) * c_phi * c_theta - g;
        
        % Euler angle derivatives
        p + q * s_phi * t_theta + r * c_phi * t_theta;
        q * c_phi - r * s_phi;
        q * s_phi * sec_theta + r * c_phi * sec_theta;
        
        % Angular velocity derivatives
        (Mp + (Jyy - Jzz) * q * r) / Jxx;
        (Mq + (Jzz - Jxx) * p * r) / Jyy;
        (Mr + (Jxx - Jyy) * p * q) / Jzz
    ];
end
