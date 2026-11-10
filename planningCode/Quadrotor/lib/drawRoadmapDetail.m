function drawRoadmapDetail(ax, F, region, opts)
    %DRAWROADMAPDETAIL Detail-mode drawing of the funnel roadmap for zoom insets.
    %   Each funnel-edge is a tube of ellipses with a filled inlet and an outlined
    %   outlet. Outlets are drawn on top of inlets, so at every node the outlet of
    %   the incoming funnel is visible inside the inlet of the next one: that
    %   nesting is the composition.
    %
    %   drawRoadmapDetail(insetAx, F, [xmin xmax ymin ymax])
    %
    %   Draw order: the whole search tree first (quiet, gray), then the whole
    %   solution funnel-path on top (all of its layers), then robot and goal.
    %
    %   Reads only planner data (no class methods are called):
    %     F.numNodes, F.graphNodes(i).{withinObstacle, bestInlet, parentFunnelEdge,
    %     parent, pose}, F.startNode, F.goalNode,
    %     F.funnelEdges(j).{trajectory_workSpace, invariantSet_workSpace}
    %   Convention: knot 1 = inlet, last knot = outlet.
    
    arguments
        ax
        F
        region (1,4) double                    % [xmin xmax ymin ymax]
        opts.EllipseEvery (1,1) double  = 3    % solution path: slice every n-th knot
        opts.TreeEvery    (1,1) double  = 8    % tree: slice every n-th knot (Inf = none)
        opts.TreeOutlets  (1,1) logical = true % tree: draw outlet outlines
        opts.Margin       (1,1) double  = 1    % include edges up to this far outside (m)
    end
    
    C  = ijrr_colors();
    th = linspace(0, 2*pi, 60);
    
    % ---- Styles: tree recedes, solution path carries the figure ---------------
    ST.tree.tube    = [0.84 0.84 0.84];
    ST.tree.slice   = [0.66 0.66 0.66];
    ST.tree.inlet   = [0.74 0.74 0.74];
    ST.tree.outlet  = [0.40 0.40 0.40];
    ST.tree.traj    = [0.50 0.50 0.50];
    ST.tree.every   = opts.TreeEvery;
    ST.tree.outlets = opts.TreeOutlets;
    ST.tree.lw      = struct('slice', 0.3, 'outlet', 0.6, 'traj', 0.5);
    
    ST.branch.tube    = 0.45 + 0.55 * C.branch;   % green tube
    ST.branch.slice   = C.branchTraj;             % dark green cross-sections
    ST.branch.inlet   = C.branch;                 % saturated green fill
    ST.branch.outlet  = [0.00 0.20 0.15];         % darkest green outline
    ST.branch.traj    = 0.55 + 0.45 * C.branch;   % light green centerline
    ST.branch.every   = opts.EllipseEvery;
    ST.branch.outlets = true;
    ST.branch.lw      = struct('slice', 0.45, 'outlet', 0.9, 'traj', 0.6);
    
    % ---- Solution branch: backtrack parent pointers (as drawGoalBranch) -------
    branchIdx = [];
    goal = F.goalNode.pose;  node = F.startNode;
    for guard = 1:F.numNodes
        if all(abs(node.pose - goal) < 0.01) || isnan(node.parent), break; end
        branchIdx(end + 1) = node.parentFunnelEdge;
        node = F.graphNodes(node.parent);
    end
    
    % ---- Tree edges near the region (as drawSearchFunnel), minus the branch ---
    treeIdx = [];
    for i = 1:F.numNodes
        nd = F.graphNodes(i);
        if nd.withinObstacle || isnan(nd.bestInlet), continue; end
        j = nd.parentFunnelEdge;
        if ismember(j, branchIdx), continue; end
        e = F.funnelEdges(j);
        if isempty(e), continue; end
        if nearRegion(e.trajectory_workSpace, region, opts.Margin)
            treeIdx(end + 1) = j;
        end
    end
    keep = arrayfun(@(j) nearRegion(F.funnelEdges(j).trajectory_workSpace, region, opts.Margin), branchIdx);
    branchIdx = branchIdx(keep);
    
    % ---- Draw: tree completely, then solution path completely on top ----------
    drawGroup(ax, F, treeIdx,   ST.tree,   th);
    drawGroup(ax, F, branchIdx, ST.branch, th);
    
    % ---- Robot and goal ---------------------------------------------------------
    r = F.startNode.pose;
    plot(ax, r(1), r(2), 'd', 'MarkerFaceColor', C.robot, 'MarkerEdgeColor', 'k', ...
        'MarkerSize', 6, 'LineWidth', 0.6);
    plot(ax, goal(1), goal(2), 'x', 'Color', C.goal, 'MarkerSize', 7, 'LineWidth', 1.5);
end

% ===================== Local functions ======================================
function drawGroup(ax, F, idx, s, th)
    % Layers within one group: tube fills -> slices -> inlet fills -> outlets -> trajectories
    for j = idx                                             % 1. tube fills
        [c, P] = edgeData(F, j);
        for k = 1:size(c, 2)
            E = ell(c(:, k), P(:, :, k), th);
            fill(ax, E(1, :), E(2, :), s.tube, 'EdgeColor', 'none');
        end
    end
    if isfinite(s.every)                                    % 2. intermediate slices
        for j = idx
            [c, P] = edgeData(F, j);
            N = size(c, 2);
            for k = 1 + s.every : s.every : N - 1
                E = ell(c(:, k), P(:, :, k), th);
                plot(ax, E(1, :), E(2, :), '-', 'Color', s.slice, 'LineWidth', s.lw.slice);
            end
        end
    end
    for j = idx                                             % 3. inlets (filled)
        [c, P] = edgeData(F, j);
        E = ell(c(:, 1), P(:, :, 1), th);
        fill(ax, E(1, :), E(2, :), s.inlet, 'EdgeColor', 'none');
    end
    if s.outlets                                            % 4. outlets (outline)
        for j = idx
            [c, P] = edgeData(F, j);
            E = ell(c(:, end), P(:, :, end), th);
            plot(ax, E(1, :), E(2, :), '-', 'Color', s.outlet, 'LineWidth', s.lw.outlet);
        end
    end
    for j = idx                                             % 5. nominal trajectories
        c = F.funnelEdges(j).trajectory_workSpace;
        plot(ax, c(1, :), c(2, :), '-', 'Color', s.traj, 'LineWidth', s.lw.traj);
    end
end

function [c, P] = edgeData(F, j)
    e = F.funnelEdges(j);
    c = e.trajectory_workSpace;
    P = e.invariantSet_workSpace;
end

function E = ell(c, P, th)
    [V, D] = eig((P + P') / 2);
    E = c(1:2) + V * diag(1 ./ sqrt(diag(D))) * [cos(th); sin(th)];
end

function tf = nearRegion(c, region, m)
    tf = any(c(1, :) >= region(1) - m & c(1, :) <= region(2) + m & ...
             c(2, :) >= region(3) - m & c(2, :) <= region(4) + m);
end