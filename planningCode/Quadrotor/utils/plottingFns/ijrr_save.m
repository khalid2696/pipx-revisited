function outPath = ijrr_save(fig, basename, fmt, opts)
%IJRR_SAVE Export a figure in a SAGE/IJRR-accepted format.
%   ijrr_save(fig, 'fig_tracking')                         % EPS, current dir
%   ijrr_save(fig, 'fig_tracking', 'eps', Folder='figures')
%   ijrr_save(fig, 'fig_cloud', 'png', Folder='../paper/figures')
%   p = ijrr_save(...)                                     % returns full path
%
%   basename has no extension; the correct one is appended.
%   Folder is created if it does not exist.
%
%   Use 'png' when the figure has transparency (FaceAlpha < 1; EPS has no
%   alpha channel), dense scatter (>~1e4 points), surfaces, or images.
%   SAGE minimums: 300 dpi for halftones, 800 dpi for raster line art.

arguments
    fig
    basename (1,:) char
    fmt      (1,:) char {mustBeMember(fmt, {'eps', 'png'})} = 'eps'
    opts.Folder (1,:) char = '.'
end

[~, ~, ext] = fileparts(basename);
if ~isempty(ext)
    error('ijrr_save:ext', 'Pass basename without an extension.');
end

if ~isfolder(opts.Folder)
    [ok, msg] = mkdir(opts.Folder);
    if ~ok
        error('ijrr_save:folder', 'Cannot create folder "%s": %s', ...
            opts.Folder, msg);
    end
end

outPath = fullfile(opts.Folder, [basename '.' fmt]);

drawnow;
switch fmt
    case 'eps'
        if hasTransparency(fig)
            warning('ijrr_save:alpha', ['Figure uses transparency; EPS ' ...
                'cannot store it. Use ''png'' or solid light fills.']);
        end
        exportgraphics(fig, outPath, 'ContentType', 'vector', ...
            'BackgroundColor', 'white');
    case 'png'
        exportgraphics(fig, outPath, 'ContentType', 'image', ...
            'Resolution', 1000, 'BackgroundColor', 'white');
end

if nargout == 0
    clear outPath
end
end

function tf = hasTransparency(fig)
objs = findall(fig, '-property', 'FaceAlpha');
tf = false;
for k = 1:numel(objs)
    a = objs(k).FaceAlpha;
    if isnumeric(a) && any(a(:) < 1)
        tf = true; return
    end
end
end