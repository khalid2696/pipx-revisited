function C = newColorScheme()
    % colorblind-safe palette (Okabe-Ito based) for all paper figures.
    %   C = newColorScheme();  plot(..., 'Color', C.pipx)
    
    % Methods (baseline comparison plots)
    C.pipx = [0.00 0.45 0.70];          % blue
    C.rrtx = [0.84 0.37 0.00];          % vermillion
    
    % Planner snapshots
    C.sensorFill       = [0.91 0.95 0.99];   % very light blue tint
    C.sensorEdge       = [0.55 0.68 0.82];
    C.obstacle         = [0.35 0.35 0.35];   % sensed: dark gray fill
    C.obstacleUnsensed = [0.45 0.45 0.45];   % not yet sensed: dashed outline
    C.tree             = [0.85 0.85 0.85];   % search-tree funnels
    C.treeTraj         = [0.40 0.40 0.40];   % search-tree nominal trajectories
    C.branch           = [0.00 0.62 0.45];   % goal-branch funnels (bluish green)
    C.branchTraj       = [0.00 0.30 0.22];   % goal-branch trajectory (dark green)
    C.robot            = [0.80 0.47 0.65];   % reddish purple
    C.goal             = [0.84 0.37 0.00];   % vermillion
end
