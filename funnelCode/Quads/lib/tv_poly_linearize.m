% Linearize polynomial dynamics about a nominal trajectory.
%
% [A,B] = tv_poly_linearize(xdot,x0,u0)
%
% Input:
%   xdot  -- xdot(t,x,u) takes (1-by-1)x(n-by-1)x(m-by-1) msspolys
%                returns n-by-1 msspoly.
%   x0    -- x0(t) function n-by-1 double to linearize around.
%   u0    -- u0(t) function m-by-1 double to linearize around.
% Output:
%   A     -- A(t) n-by-n Jacobian of xdot w.r.t. x at x0(t).
%   B     -- B(t) n-by-m Jacobian of xdot w.r.t. u at u0(t).
function [A,B] = tv_poly_linearize(f,x0,u0)
    t = msspoly('t',1);
    if size(x0(0),2) ~= 1, error('x0 must be a column.'); end
    n = length(x0(0)); 
    x = msspoly('x',n);
    
    
    if nargin < 3
        if nargout > 1, error(['u0 must be provided to output B ' ...
                               'matrix.']); end
        xdot = f(t,x);
        Af = sim_p2f(diff(xdot,x),[t;x]);
        A = @(t) Af([t;x0(t)]);
    else
        if size(u0(0),2) ~= 1, error('u0 must be a column.'); end
        m = length(u0(0));
        u = msspoly('u',m);
    
        xdot = f(t,x,u);
        Af = sim_p2f(diff(xdot,x),[t;x;u]);
        A = @(t) Af([t;x0(t);u0(t)]);
        Bf = sim_p2f(diff(xdot,u),[t;x;u]);
        B = @(t) Bf([t;x0(t);u0(t)]);
    end
end
