clc; clearvars; close all

%% checkFunnelPath_MCRollouts.m
%  Monte Carlo verification of a funnel-path (sequence of funnels) for a
%  12-state quadrotor under TVLQR feedback.
%
%  The planner builds the nominal (x_nom,u_nom) with TRAPEZOIDAL COLLOCATION
%     x_{k+1} - x_k = (H/2)(f_k + f_{k+1})
%  on the original knots, so x_nom is NOT a forward-RK4 trajectory of f. This
%  script therefore propagates in DEVIATION coordinates e = x - x_nom with an
%  IMPLICIT TRAPEZOID step anchored to the stored x_nom:
%     - on-nominal e=0 stays e=0 exactly (funnel value = 0 at zero perturbation)
%     - implicit trapezoid is A-stable, so coarse knot steps stay bounded
%     - perturbed deviations are checked against the same discrete closed loop
%       the invariance certificate was built on
%  No upsampling: containment is verified at the certified knots.
%
%  For each of num_samples rollouts we propagate from the start of the path to
%  its end, injecting n state-perturbations per funnel (n*p total) at
%  stratified-random knots, each kept inside { e : e'P e <= level } (level<=1).
%
%  Verified at each knot:
%    (1) funnel containment        : e'P e <= 1
%    (2) handoff inlet containment  : carried e inside child funnel inlet
%    (3) collision                  : x([1 2]) vs. 2D circular obstacles
%
%  Inputs (loaded/defined by a wrapper, or below):
%    funnelPath{fi} struct with .time, .trajectory_stateSpace (x_nom),
%      .invariantSet_stateSpace (P, level<=1), .feedforwardControlInputs (u_nom),
%      .feedbackControlGains (K), optional .trajectory_workSpace / _workSpace.
%    quadParameters (.m,.g,.J)  REQUIRED.   obstacles  M x 3 [xc yc r]  optional.

%% Add directories
addpath('./lib/');

load('./temp_invariance_verification/sampleFunnelPath.mat'); % must contain funnelPath

% Quadrotor Parameters (imported from funnel computation code) [!!keep both in sync!]
quadParameters.m = 0.7;                  % mass (kg)
quadParameters.g = 9.81;                 % gravity (m/s^2)
quadParameters.J = [2e-3, 2e-3, 3.5e-3]; % moment of inertia (kg m^2)

%% ------------------------------------------------------------------------
%  Inputs / parameters (inherited from wrapper if present, else defaulted)
%  ------------------------------------------------------------------------
if ~exist('funnelPath','var')
    error('checkFunnelPath:noPath', 'funnelPath (ordered cell array of funnels) is required.');
end
if ~exist('quadParameters','var')
    error('checkFunnelPath:noParams', 'quadParameters (.m, .g, .J) is required.');
end

if ~exist('num_samples','var'),      num_samples = 100;         end % # rollouts
if ~exist('n_perturb','var'),        n_perturb   = 2;           end % perturbations PER funnel
if ~exist('perturbMode','var'),      perturbMode      = 'additive'; end % 'additive'|'resample'
if ~exist('perturbPlacement','var'), perturbPlacement = 'interior'; end % 'boundary'|'interior'
if ~exist('perturbLevel','var'),     perturbLevel     = 0.05;    end % level c in (0,1], boundary=1
if ~exist('divergeTol','var'),       divergeTol  = 1e6;         end % |e| beyond this (or non-finite) => diverged
if ~exist('knotRange','var'),        knotRange   = [1 Inf];     end % [kmin kmax] for perturb knots
if ~exist('robotRadius','var'),      robotRadius = 0.0;         end % obstacle inflation (0 = none)
if ~exist('containTol','var'),       containTol  = 1e-6;        end % slack on the <=1 test
if ~exist('angleIdx','var'),         angleIdx    = [7 8 9];     end % Euler-angle indices: deviations wrapped
if ~exist('newtonTol','var'),        newtonTol   = 1e-10;       end % implicit-trapezoid Newton tolerance
if ~exist('newtonMaxIter','var'),    newtonMaxIter = 25;        end % implicit-trapezoid Newton cap

hasObstacles = exist('obstacles','var') && ~isempty(obstacles);
if ~hasObstacles
    warning('checkFunnelPath:noObstacles', 'No obstacles supplied -> collision check skipped.');
    obstacles = zeros(0,3);
end

dynamics = @(x,u) quadrotor_dynamics(x, u, quadParameters);

p       = numel(funnelPath);
n_x     = size(funnelPath{1}.trajectory_stateSpace, 1);
n_u     = size(funnelPath{1}.feedforwardControlInputs, 1);
ws_dims = [1 2];

%% ------------------------------------------------------------------------
%  Concatenated (coarse) knot timeline for recording/plotting. Consecutive
%  funnels share a boundary knot -> keep funnel-1 knots 1..N, others 2..N.
%  ------------------------------------------------------------------------
[globalTime, knotMap, x_nom_global, u_nom_global] = build_coarse_timeline(funnelPath);
T = numel(globalTime);

% planner's own trapezoidal-collocation residual (disclosed floor, not a bug)
collocFloor = nominal_colloc_residual(funnelPath, dynamics, angleIdx);

fprintf('Funnel path: %d funnels, %d knots, horizon %.3f s.  Nominal collocation floor (P-metric): %.4g\n', ...
    p, T, globalTime(end), collocFloor);

%% ------------------------------------------------------------------------
%  Monte Carlo rollouts
%  ------------------------------------------------------------------------
trajectories = cell(num_samples, 1);
input_traj   = cell(num_samples, 1);
funnelValue  = zeros(num_samples, T);
inFunnel     = false(num_samples, T);
collided     = false(num_samples, 1);
firstExitT   = nan(num_samples, 1);
firstHitT    = nan(num_samples, 1);
handoffOK    = true(num_samples, max(p-1,1));
diverged     = false(num_samples, 1);

for s = 1:num_samples
    [X, U, V, hOK, dvg] = rollout_once(funnelPath, dynamics, ...
        n_perturb, perturbMode, perturbPlacement, perturbLevel, knotRange, ...
        divergeTol, angleIdx, newtonTol, newtonMaxIter);

    diverged(s)     = dvg;
    trajectories{s} = X;
    input_traj{s}   = U;
    funnelValue(s,:)= V;
    handoffOK(s,:)  = hOK;

    inside        = V <= 1 + containTol;
    inFunnel(s,:) = inside;
    exitIdx       = find(~inside, 1, 'first');
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
fprintf('Knots outside funnel             : %.2f%%\n', 100*knotOutRate);
fprintf('Handoff-inlet violations         : %d rollout(s)\n', nHandoffFail);
if any(diverged)
    fprintf(2, 'Diverged / Newton failed         : %d rollout(s)\n', sum(diverged));
end
fprintf('Max funnel value observed        : %.4g  (level = 1)\n', max(funnelValue(:), [], 'omitnan'));
fprintf('Nominal collocation floor        : %.4g  (planner residual, not a rollout error)\n', collocFloor);
if hasObstacles
    fprintf('Rollouts colliding w/ obstacle   : %d / %d  (%.1f%%)\n', ...
            nCollided, num_samples, 100*nCollided/num_samples);
    if any(~isnan(firstHitT)), fprintf('Earliest collision time          : %.3f s\n', min(firstHitT)); end
end
if any(~isnan(firstExitT))
    fprintf('Earliest funnel-exit time        : %.3f s\n', min(firstExitT));
end
fprintf('======================================================\n\n');

%% ------------------------------------------------------------------------
%  Save trajectory data + normalized Lyapunov values (e'Pe, boundary = 1)
%  ------------------------------------------------------------------------
if ~exist('saveResults','var'), saveResults = true; end
if ~exist('outputFile','var'),  outputFile  = 'funnelPath_MCRollouts.mat'; end

if saveResults
    rollout = struct();
    rollout.time        = globalTime;
    rollout.states      = cat(3, trajectories{:});
    rollout.inputs      = cat(3, input_traj{:});
    rollout.lyapunov    = funnelValue;
    rollout.inFunnel    = inFunnel;
    rollout.collided    = collided;
    rollout.firstExitT  = firstExitT;
    rollout.firstHitT   = firstHitT;
    rollout.handoffOK   = handoffOK;
    rollout.diverged    = diverged;
    rollout.x_nom       = x_nom_global;
    rollout.u_nom       = u_nom_global;
    rollout.knotMap     = knotMap;
    rollout.obstacles   = obstacles;
    rollout.collocFloor = collocFloor;
    rollout.config      = struct( ...
        'num_samples', num_samples, 'n_perturb', n_perturb, ...
        'perturbMode', perturbMode, 'perturbPlacement', perturbPlacement, ...
        'perturbLevel', perturbLevel, 'robotRadius', robotRadius, ...
        'integrator', 'implicit-trapezoid (deviation coords)');

    save(outputFile, 'rollout', '-v7.3');
    fprintf('Saved trajectory data + normalized Lyapunov values -> %s\n\n', outputFile);
end

%% ------------------------------------------------------------------------
%  Visualization
%  ------------------------------------------------------------------------
disp('Plotting workspace, funnel value, and input profiles...'); disp(' ');

plot_workspace(trajectories, funnelPath, obstacles, robotRadius, ws_dims, collided);
plot_funnel_value(funnelValue, globalTime, inFunnel);
plot_input_profiles(input_traj, globalTime, u_nom_global);

%% ========================================================================
%  Local functions
%  ========================================================================

function [X, U, V, handoffOK, diverged] = rollout_once(funnelPath, dynamics, ...
        n_perturb, mode, placement, level, knotRange, divergeTol, angleIdx, nTol, nMax)
    % One rollout in DEVIATION coordinates e = x - x_nom, marched knot-to-knot
    % with an implicit trapezoid anchored to the stored nominal. Perturbations
    % set e inside the knot ellipsoid. Records x = x_nom + e at every knot.

    p   = numel(funnelPath);
    n_x = size(funnelPath{1}.trajectory_stateSpace, 1);
    n_u = size(funnelPath{1}.feedforwardControlInputs, 1);

    T = 0;
    for fi = 1:p
        Nf = size(funnelPath{fi}.trajectory_stateSpace, 2);
        if fi == 1, T = T + Nf; else, T = T + Nf - 1; end
    end
    X = nan(n_x, T); U = nan(n_u, T); V = nan(1, T);
    handoffOK = true(1, max(p-1,1));
    diverged  = false;
    col = 0;

    e = zeros(n_x, 1);   % deviation; start exactly on nominal

    for fi = 1:p
        F  = funnelPath{fi};
        t  = F.time(:).';
        Nf = numel(t);
        xn = F.trajectory_stateSpace;
        un = F.feedforwardControlInputs;
        K  = F.feedbackControlGains;
        P  = F.invariantSet_stateSpace;

        kmin = max(1, knotRange(1));
        kmax = min(Nf, knotRange(2));
        pk   = pick_perturbation_knots(kmin, kmax, n_perturb);

        for k = 1:Nf
            % handoff: carried deviation vs child inlet (seams match -> e carries directly)
            if fi > 1 && k == 1
                dd = wrap_dev(e, angleIdx);
                if dd.' * P(:,:,1) * dd > 1 + 1e-6, handoffOK(fi-1) = false; end
            end

            % perturbation: set deviation inside this knot's ellipsoid
            if any(k == pk)
                xp = perturb_within_ellipsoid(xn(:,k) + e, xn(:,k), P(:,:,k), mode, placement, level, angleIdx);
                e  = wrap_dev(xp - xn(:,k), angleIdx);
            end

            % control + record (skip duplicate shared boundary knot for fi>1)
            u = un(:,k) - K(:,:,k) * wrap_dev(e, angleIdx);
            if ~(fi > 1 && k == 1)
                col      = col + 1;
                dd       = wrap_dev(e, angleIdx);
                X(:,col) = xn(:,k) + e;
                U(:,col) = u;
                V(col)   = dd.' * P(:,:,k) * dd;
            end

            % propagate deviation to the next knot (implicit trapezoid, A-stable)
            if k < Nf
                H = t(k+1) - t(k);
                [e, ok] = trapezoid_dev_step(dynamics, e, H, ...
                    xn(:,k),   un(:,k),   K(:,:,k), ...
                    xn(:,k+1), un(:,k+1), K(:,:,k+1), angleIdx, nTol, nMax);
                if ~ok || any(~isfinite(e)) || norm(e) > divergeTol
                    diverged = true; return;
                end
            end
        end
    end
end

function [e1, ok] = trapezoid_dev_step(dynamics, e0, H, ...
        xn0, un0, K0, xn1, un1, K1, angleIdx, tol, maxit)
    % Implicit trapezoid on the deviation dynamics, matching the planner's
    % collocation:  e1 - e0 = (H/2)(df0 + df1),  dfj = f(xnj+ej, unj - Kj*wrap(ej)) - f(xnj,unj).
    % Modified Newton (Jacobian formed once at the predictor).
    dfun = @(e, xn, un, Kk) dynamics(xn + e, un - Kk*wrap_dev(e, angleIdx)) - dynamics(xn, un);
    df0  = dfun(e0, xn0, un0, K0);                    % fixed over the step
    G    = @(e1) e1 - e0 - (H/2)*(df0 + dfun(e1, xn1, un1, K1));

    e1 = e0;                                          % predictor (e=0 on nominal -> G(e0)=0)
    Gv = G(e1);
    J  = fd_jac(G, e1, Gv);
    ok = false;
    for it = 1:maxit
        if norm(Gv) < tol, ok = true; break; end
        e1 = e1 - J \ Gv;
        Gv = G(e1);
    end
    if norm(Gv) < tol, ok = true; end
end

function J = fd_jac(fun, x, f0)
    n = numel(x); J = zeros(n); h = 1e-6;
    for i = 1:n
        xp = x; xp(i) = xp(i) + h;
        J(:,i) = (fun(xp) - f0) / h;
    end
end

function r = nominal_colloc_residual(funnelPath, dynamics, angleIdx)
    % Max trapezoidal-collocation residual of the stored nominal, in the
    % P-metric. This is a planner property (solver tolerance / discretization),
    % reported for disclosure; the deviation rollout does not incur it.
    r = 0;
    for fi = 1:numel(funnelPath)
        F = funnelPath{fi}; t = F.time; N = numel(t);
        xn = F.trajectory_stateSpace; un = F.feedforwardControlInputs;
        P  = F.invariantSet_stateSpace;
        for k = 1:N-1
            H  = t(k+1) - t(k);
            f1 = dynamics(xn(:,k),   un(:,k));
            f2 = dynamics(xn(:,k+1), un(:,k+1));
            d  = xn(:,k+1) - xn(:,k) - (H/2)*(f1 + f2);
            d(angleIdx) = mod(d(angleIdx) + pi, 2*pi) - pi;
            r = max(r, d.' * P(:,:,k+1) * d);
        end
    end
end

function [globalTime, knotMap, x_nom_global, u_nom_global] = build_coarse_timeline(funnelPath)
    p = numel(funnelPath);
    globalTime = []; knotMap = []; x_nom_global = []; u_nom_global = [];
    tOffset = 0;
    for fi = 1:p
        F  = funnelPath{fi};
        t  = F.time(:).'; Nf = numel(t);
        xn = F.trajectory_stateSpace; un = F.feedforwardControlInputs;
        if fi == 1, kStart = 1; else, kStart = 2; end
        for k = kStart:Nf
            globalTime(end+1)     = tOffset + t(k);   %#ok<AGROW>
            knotMap(end+1,:)      = [fi, k];          %#ok<AGROW>
            x_nom_global(:,end+1) = xn(:,k);          %#ok<AGROW>
            u_nom_global(:,end+1) = un(:,k);          %#ok<AGROW>
        end
        tOffset = tOffset + t(end);
    end
end

function pk = pick_perturbation_knots(kmin, kmax, n)
    % Stratified: split [kmin,kmax] into n bins, one random knot each.
    if n <= 0, pk = []; return; end
    span = kmax - kmin + 1;
    n    = min(n, span);
    edges = round(linspace(kmin, kmax + 1, n + 1));
    pk = zeros(1, n);
    for b = 1:n
        lo = edges(b); hi = max(lo, edges(b+1) - 1);
        pk(b) = randi([lo, hi]);
    end
    pk = unique(pk);
end

function x_pert = perturb_within_ellipsoid(x_cur, x_nom, P, mode, placement, level, angleIdx)
    % Perturb so the result lies in { d : d'Pd <= level } (level<=1), shaped by
    % the ellipsoid. mode: 'resample' (fresh deviation) | 'additive' (kick current).
    % placement: 'boundary' (on the level surface) | 'interior' (anywhere inside).
    n  = numel(x_nom);
    A  = ellipsoid_map(P);
    yd = randn(n,1); yd = yd / norm(yd);

    switch lower(placement)
        case 'boundary', c = level;
        case 'interior', c = level * (rand^(1/n))^2;
        otherwise, error('perturb:placement', 'Unknown placement "%s".', placement);
    end

    switch lower(mode)
        case 'resample'
            d = A * (sqrt(c) * yd);
        case 'additive'
            d = wrap_dev(x_cur - x_nom, angleIdx) + A * (sqrt(c) * yd);
            v = d.' * P * d;
            if strcmpi(placement, 'boundary')
                d = d * sqrt(level / v);
            elseif v > level
                d = d * sqrt(level / v);
            end
        otherwise, error('perturb:mode', 'Unknown mode "%s".', mode);
    end
    x_pert = x_nom + d;
end

function A = ellipsoid_map(P)
    % A = P^{-1/2}: maps the unit ball onto { d : d'Pd <= 1 }.
    P = (P + P.') / 2;
    [Vv, Dv] = eig(P);
    d = max(real(diag(Dv)), 1e-12);
    A = Vv * diag(1 ./ sqrt(d)) * Vv.';
end

function dw = wrap_dev(d, angleIdx)
    % Wrap Euler-angle components of a deviation to [-pi, pi].
    dw = d;
    dw(angleIdx) = mod(dw(angleIdx) + pi, 2*pi) - pi;
end

function [hit, hitIdx] = check_collisions(xy, obstacles, robotRadius)
    hit = false; hitIdx = NaN;
    for t = 1:size(xy,2)
        pxy = xy(:,t);
        if any(~isfinite(pxy)), continue; end
        for o = 1:size(obstacles,1)
            c = obstacles(o,1:2).'; r = obstacles(o,3) + robotRadius;
            if sum((pxy - c).^2) <= r^2, hit = true; hitIdx = t; return; end
        end
    end
end

function plot_workspace(trajectories, funnelPath, obstacles, robotRadius, ws_dims, collided)
    figure; hold on; grid on; axis equal;
    for o = 1:size(obstacles,1)
        draw_circle(obstacles(o,1), obstacles(o,2), obstacles(o,3), [0.85 0.33 0.33], 0.35);
        if robotRadius > 0
            draw_circle(obstacles(o,1), obstacles(o,2), obstacles(o,3)+robotRadius, [0.85 0.33 0.33], 0.10);
        end
    end
    for fi = 1:numel(funnelPath)
        if ~isfield(funnelPath{fi},'invariantSet_workSpace') || isempty(funnelPath{fi}.invariantSet_workSpace), continue; end
        xw = funnelPath{fi}.trajectory_workSpace; Mw = funnelPath{fi}.invariantSet_workSpace;
        step = max(1, round(size(xw,2)/6));
        for k = 1:step:size(xw,2)
            eb = ellipsoid_map(Mw(:,:,k)) * [cos(linspace(0,2*pi,60)); sin(linspace(0,2*pi,60))];
            plot(xw(1,k)+eb(1,:), xw(2,k)+eb(2,:), '-', 'Color', [0.6 0.6 0.6 0.5], 'LineWidth', 0.5);
        end
        plot(xw(1,:), xw(2,:), 'k--', 'LineWidth', 1.5);
    end
    for s = 1:numel(trajectories)
        X = trajectories{s}; col = 'b'; if collided(s), col = 'r'; end
        plot(X(ws_dims(1),:), X(ws_dims(2),:), '-', 'Color', col, 'LineWidth', 0.4);
    end
    xlabel('p_x'); ylabel('p_y'); title('MC Funnel-Path Rollouts (workspace)');
end

function draw_circle(xc, yc, r, faceColor, alpha)
    th = linspace(0, 2*pi, 100);
    fill(xc + r*cos(th), yc + r*sin(th), faceColor, 'EdgeColor', faceColor*0.7, 'FaceAlpha', alpha);
end

function plot_funnel_value(funnelValue, time, inFunnel)
    figure; hold on; grid on;
    for s = 1:size(funnelValue,1)
        col = [0.5 0.5 0.9 0.4];
        if ~all(inFunnel(s,:)), col = [0.9 0.3 0.3 0.6]; end
        plot(time, funnelValue(s,:), '-', 'Color', col, 'LineWidth', 0.4);
    end
    plot(time, mean(funnelValue,1,'omitnan'), 'k-', 'LineWidth', 2);
    yline(1, 'r--', 'funnel level = 1', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('(x-x_{nom})^T P (x-x_{nom})');
    title('Funnel Containment Value Over Time'); xlim([time(1) time(end)]);
end

function plot_input_profiles(input_traj, time, u_nom_global)
    m = size(u_nom_global, 1); figure; labels = {'T','M_x','M_y','M_z'};
    for i = 1:m
        subplot(m,1,i); hold on; grid on;
        for s = 1:numel(input_traj), plot(time, input_traj{s}(i,:), 'b-', 'LineWidth', 0.3); end
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
