% Time Invariant Domain of Attraction Estimate
%
% rho = ti_poly_doa(x,f,V,Lmonom)
% 
% Input:
%    x  -- an n-by-1 of free msspoly (the state variables)
%    f  -- an n-by-1 msspoly of differential equations (\dot x)
%    V  -- a positive definite candidate Lyapunov function.
%    Lmonom -- monomials for the LaGrange multiplier.
%
% Returns:
%    rho -- a conservative quantity such that { x | V(x) < rho}
%           is in the domain of attraction of the origin.
%
% Inspired by Parillo '05.
%
function rho = ti_poly_roa(x,xdot,V,Lmonom)
    x = msspoly(x); xdot = msspoly(xdot); V = msspoly(V);
    if size(x,2) ~= 1, error('x must be a column'); end
    if size(xdot,2) ~= 1, error('f must be a column'); end
    if size(x,1) ~= size(xdot,1),
        error('x and xdot not matching sizes'); 
    end
    if size(V) ~= [1 1], error('V must be 1-by-1'); end
    
    prog = mssprog;
    
    Vdot = diff(V,x)*xdot;
    
    if nargin < 4 % Just a guess.
        Lmonom = monomials(x,0:deg(Vdot,x));
    end
    
    rho = msspoly('r');
    prog.free = rho;
    
    [prog,l] = new(prog,length(Lmonom),'free');
    L = l'*Lmonom;
    
    prog.sos = (x'*x)*(V - rho) +  L*Vdot;

    prog.sedumi = -rho;
    
    rho = double(prog(rho));
end