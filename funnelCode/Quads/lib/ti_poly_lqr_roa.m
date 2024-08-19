% Time Invariant LQR Domain of Attraction for Polynomial Dynamics
%
% [K,S,rho] = ti_poly_lqr_doa(f,x0,u0,Q,R)
%
% Inputs:
%    f  -- Function xdot = f(x,u) 
%               x - n-by-1 msspoly, u m-by-1 msspoly
%               xdot - n-by-1 msspoly.
%    x0 -- n-by-1 equilibrium to be stabilized.
%    u0 -- m-by-1 nominal command.
%    Q  -- Regulator cost on state: (x-x0)'Q(x-x0)
%    R  -- Regulator cost on action: (u-u0)'R(u-u0)
%
% Returns:
%    K   -- feedback gain.
%    S   -- Cost-to-go matrix for the linearized system.
%    rho -- Conserviate rho such that { x | (x-x0)'S(x-x0) <= rho}
%           is positively invariant.
%
function [K,S,rho] = ti_poly_lqr_roa(f,x0,u0,Q,R)
    if size(x0,2) ~= 1, error('x0 must be a column.'); end
    if size(u0,2) ~= 1, error('u0 must be a column.'); end
    n = length(x0); m = length(u0);
    if size(Q) ~= [n n], error('Q must be n-by-n.'); end
    if size(R) ~= [m m], error('R must be m-by-m.'); end
    
    x = msspoly('x',n);
    u = msspoly('u',m);
    xdot = msspoly(f(x,u));

    if norm(double(subs(xdot,[x;u],[x0;u0])),1) > 1e-16
        error('(x0,u0) is not an equilibrium');
    end
    
    AB = double(subs(diff(xdot,[x;u]),[x;u],[x0;u0]));
    A = AB(1:n,1:n);
    B = AB(1:n,n+(1:m));
    
    [K,S] = lqr(A,B,Q,R);
    xbar = x;
    x = xbar+x0;
    rho = ti_poly_roa(xbar,f(x,u0-K*xbar),...
                      xbar'*S*xbar);
    
    % Now change coordinates.
    %T = inv(balqf(S,(A'*S+S*A)/2));
    %Tinvx = x;
    %xbar = T*Tinvx;
    %x = xbar+x0;
    %xbardot = T*f(x,u0-K*xbar);
    %V = xbar'*S*xbar;

    %rho = ti_poly_roa(Tinvx,xbardot,...
    %                       V);
end