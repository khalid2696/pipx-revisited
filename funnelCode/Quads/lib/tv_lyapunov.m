function [ts,Ss] = tv_lyapunov(tspan,A,Q,Sf)
    if size(tspan) ~= [1 2], error('tspan must = [t0 tfinal'); end
    
    t0 = tspan(1);
    
    n = size(A(t0),1); 
    if size(A(t0)) ~= [n n], error('A must be n-by-n'); end
    if size(Sf) ~= [n n], error('Qf must be n-by-n'); end
    if size(Q(t0)) ~= [n n], error('Q must be n-by-n'); end

    [ts,Ss] = matrixODE(@ode45,...
                        @(t,S) -A(t)'*S-S*A(t)-Q(t),...
                        fliplr(tspan),Sf); %diff. Riccati equation
end