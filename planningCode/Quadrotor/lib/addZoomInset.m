function insetAx = addZoomInset(ax, center, opts)
    %ADDZOOMINSET Zoom box in AX, magnified inset inside the same panel, and two
    %   connector lines. Call after the panel is fully drawn (title included),
    %   BEFORE the planner advances to the next snapshot.
    %
    %   insetAx = addZoomInset(ax, robotXY, Avoid=goalXY, ...
    %                 DrawFcn=@(a, region) drawInsetScene(a, F, W, region))
    %
    %   Geometry is computed in inches from the axes' final inner position, and
    %   the zoom box and connectors are drawn in AX's data coordinates.
    %
    %   Options
    %     HalfWidth : zoom window half-width (m), default 7
    %     Clamp     : keep the zoom window inside the axes limits, default true
    %     Size      : inset width as a fraction of the plot-box width, default 0.45
    %     Corner    : 'auto' or 'NE','NW','SE','SW'
    %     Avoid     : N x 2 data points the inset must not cover (e.g. the goal)
    %     Margin    : gap between inset and plot-box edge (fraction of width)
    %     Color     : zoom box / inset frame / connector color
    %     DrawFcn   : DrawFcn(insetAx, region) or DrawFcn(insetAx)
    
    arguments
        ax
        center (1,2) double
        opts.HalfWidth (1,1) double  = 7
        opts.Clamp     (1,1) logical = true
        opts.Size      (1,1) double  = 0.45
        opts.Corner    (1,:) char {mustBeMember(opts.Corner, {'auto','NE','NW','SE','SW'})} = 'auto'
        opts.Avoid     (:,2) double  = zeros(0, 2)
        opts.Margin    (1,1) double  = 0.02
        opts.Color     (1,3) double  = [0.30 0.30 0.30]
        opts.DrawFcn   = []
    end
    
    if isgraphics(ax, 'figure'), ax = ax.CurrentAxes; end
    if ~isgraphics(ax, 'axes')
        error('addZoomInset:notAxes', 'First argument must be the panel''s axes handle.');
    end
    fig = ancestor(ax, 'figure');
    
    % ---- Freeze the main axes layout before measuring it ----------------------
    % With the default 'outerposition' constraint, the title and labels move the
    % plot box after they are added; measure only once the layout has settled.
    drawnow;
    oldUnits = ax.Units;  ax.Units = 'inches';
    ax.PositionConstraint = 'innerposition';
    pos = ax.Position;
    ax.Units = oldUnits;
    
    XL = ax.XLim;  YL = ax.YLim;  dx = diff(XL);  dy = diff(YL);
    
    % ---- Zoom window (optionally kept inside the axes) -------------------------
    hw = opts.HalfWidth;
    c  = center;
    if opts.Clamp
        c(1) = min(max(c(1), XL(1) + hw), XL(2) - hw);
        c(2) = min(max(c(2), YL(1) + hw), YL(2) - hw);
    end
    zx = c(1) + [-hw hw];
    zy = c(2) + [-hw hw];
    region = [zx zy];
    
    % ---- Plot box of AX in inches ------------------------------------------------
    if strcmp(ax.DataAspectRatioMode, 'manual')
        s  = min(pos(3) / dx, pos(4) / dy);
        bw = s * dx;  bh = s * dy;
        B  = [pos(1) + (pos(3) - bw) / 2, pos(2) + (pos(4) - bh) / 2, bw, bh];
    else
        B  = pos;
    end
    d2i = @(p) [B(1) + (p(1) - XL(1)) / dx * B(3), B(2) + (p(2) - YL(1)) / dy * B(4)];
    i2d = @(q) [XL(1) + (q(1) - B(1)) / B(3) * dx, YL(1) + (q(2) - B(2)) / B(4) * dy];
    
    % ---- Inset rectangle: candidate corners --------------------------------------
    iw = opts.Size * B(3);
    m  = opts.Margin * B(3);
    R.NE = [B(1)+B(3)-m-iw, B(2)+B(4)-m-iw, iw, iw];
    R.NW = [B(1)+m,         B(2)+B(4)-m-iw, iw, iw];
    R.SE = [B(1)+B(3)-m-iw, B(2)+m,         iw, iw];
    R.SW = [B(1)+m,         B(2)+m,         iw, iw];
    
    corner = opts.Corner;
    if strcmp(corner, 'auto')
        zb = [d2i([zx(1) zy(1)]), d2i([zx(2) zy(2)]) - d2i([zx(1) zy(1)])];   % zoom box, inches
        names = {'NE','NW','SE','SW'};  best = inf;
        for k = 1:4
            P = R.(names{k});
            score = 1e3 * rectOverlap(P, zb);                        % never cover the zoom box
            for a = 1:size(opts.Avoid, 1)                            % avoid listed points
                q = d2i(opts.Avoid(a, :));
                score = score + 1e2 * inRect(q, P, 0.05 * iw);
            end
            score = score - norm(P(1:2) + P(3:4)/2 - d2i(c));        % prefer far from robot
            if score < best, best = score; corner = names{k}; end
        end
    end
    P = R.(corner);
    
    % ---- Zoom box and connectors, in AX data coordinates ---------------------------
    lineOpts = {'Color', opts.Color, 'Clipping', 'off', 'HandleVisibility', 'off'};
    plot(ax, zx([1 2 2 1 1]), zy([1 1 2 2 1]), '-', 'LineWidth', 0.6, lineOpts{:});
    
    iNW = i2d([P(1) P(2)+P(4)]);  iNE = i2d([P(1)+P(3) P(2)+P(4)]);
    iSW = i2d([P(1) P(2)]);       iSE = i2d([P(1)+P(3) P(2)]);
    bNW = [zx(1) zy(2)];  bNE = [zx(2) zy(2)];
    bSW = [zx(1) zy(1)];  bSE = [zx(2) zy(1)];
    switch corner                                    % non-crossing corner pairs
        case {'NE','SW'}, pairs = {bNW, iNW; bSE, iSE};
        case {'NW','SE'}, pairs = {bNE, iNE; bSW, iSW};
    end
    for k = 1:2
        a = pairs{k, 1};  b = pairs{k, 2};
        plot(ax, [a(1) b(1)], [a(2) b(2)], '-', 'LineWidth', 0.5, lineOpts{:});
    end
    
    % ---- Inset axes ------------------------------------------------------------------
    insetAx = axes(fig, 'Units', 'inches', 'Position', P, 'PositionConstraint', 'innerposition');
    set(fig, 'CurrentAxes', insetAx);
    hold(insetAx, 'on');
    if ~isempty(opts.DrawFcn)
        if nargin(opts.DrawFcn) >= 2, opts.DrawFcn(insetAx, region);
        else,                         opts.DrawFcn(insetAx); end
    end
    xlim(insetAx, zx);  ylim(insetAx, zy);
    daspect(insetAx, [1 1 1]);
    insetAx.XTick = [];  insetAx.YTick = [];
    insetAx.Box = 'on';  insetAx.Layer = 'top';  insetAx.Color = 'w';
    insetAx.XColor = opts.Color;  insetAx.YColor = opts.Color;
    insetAx.LineWidth = 0.7;
    set(fig, 'CurrentAxes', ax);
end

% ===================== Local functions =======================================
function a = rectOverlap(P, Q)
    % Overlap area of two [x y w h] rectangles
    w = min(P(1)+P(3), Q(1)+Q(3)) - max(P(1), Q(1));
    h = min(P(2)+P(4), Q(2)+Q(4)) - max(P(2), Q(2));
    a = max(w, 0) * max(h, 0);
end

function tf = inRect(q, P, pad)
    tf = q(1) >= P(1)-pad && q(1) <= P(1)+P(3)+pad && q(2) >= P(2)-pad && q(2) <= P(2)+P(4)+pad;
end