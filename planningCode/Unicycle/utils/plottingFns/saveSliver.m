function saveSliver(fig, basename, opts)
%SAVESLIVER Export a sliverFig snapshot cropped to an identical box every time.
%   saveSliver(fig, './paper_figures/corridor_t1')
%
%   Renders the figure, locates the axes frame in the image, and crops a
%   fixed margin around it. Every sliver therefore has the same pixel size
%   (for equal window lengths) regardless of window or screen effects.
%   The PNG carries its dpi, so LaTeX places it at true physical size.

arguments
    fig
    basename (1,:) char
    opts.DPI     (1,1) double = 1000
    opts.LabelW  (1,1) double = 0.20   % must match LABEL_W in sliverFig (in)
    opts.Pad     (1,1) double = 0.04   % must match PAD in sliverFig (in)
end

drawnow;
img = print(fig, '-RGBImage', sprintf('-r%d', opts.DPI));

% Axes frame: long runs of near-black pixels (frame is 15% gray; obstacles,
% tree, and fills are all lighter)
g    = mean(double(img), 3);
dark = g < 70;
cols = find(mean(dark, 1) > 0.5);
rows = find(mean(dark, 2) > 0.5);
if isempty(cols) || isempty(rows)
    error('saveSliver:frame', 'Axes frame not found; is ax.Box ''on''?');
end
left = min(cols);  right  = max(cols);
top  = min(rows);  bottom = max(rows);

% Fixed margins around the frame
mL = round(opts.LabelW * opts.DPI);
mP = round(opts.Pad    * opts.DPI);
r1 = top - mP;   r2 = bottom + mP;
c1 = left - mL;  c2 = right + mP;

% Pad with white if the render is tighter than the requested box
padT = max(0, 1 - r1);  padB = max(0, r2 - size(img,1));
padL = max(0, 1 - c1);  padR = max(0, c2 - size(img,2));
img  = padarray(img, [padT padL], 255, 'pre');
img  = padarray(img, [padB padR], 255, 'post');
out  = img((r1:r2) + padT, (c1:c2) + padL, :);

ppm = round(opts.DPI / 0.0254);    % pixels per meter, for LaTeX sizing
imwrite(out, [basename '.png'], 'ResolutionUnit', 'meter', ...
    'XResolution', ppm, 'YResolution', ppm);
end
