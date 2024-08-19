% Piecewise Polynomial to msspoly
%
% p = sim_pp2p(t,s)
%
% Input:
%     t -- 1-by-1 msspoly (variable of spline)
%     s -- pp object of n-by-1 with N pieces.
% Output:
%     p -- n-by-N msspoly in t. Each column a piece.
%
% See Also: sim_ps2pp
%
function p = sim_pp2p(t,s)
    N = s.pieces;
    if nargin < 3, pieces = 1:N; end
    if size(pieces,1) ~= 1, error('pieces must be a row');end
    
    o = s.order;
    if size(s.dim) ~= [1 1], 
        error('Can only transform n-by-1 pieces.');
    end
    n = s.dim; 
    M = length(pieces);
    I = repmat((1:n)',1,M)+n*repmat(pieces-1,n,1);
    m = monomials(t,0:o-1);
    p = reshape(s.coefs(I,:)*m(o:-1:1),n,M);
end