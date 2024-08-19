function [ts,Ss] = tv_lqr_riccati(tspan,A,B,Q,R,Qf)
    if size(tspan) ~= [1 2], error('tspan must = [t0 tfinal'); end
    
    t0 = tspan(1);
    
    n = size(A(t0),1); m = size(B(t0),2);
    if size(A(t0)) ~= [n n], error('A must be n-by-n'); end
    if size(Qf) ~= [n n], error('Qf must be n-by-n'); end
    if size(Q(t0)) ~= [n n], error('Q must be n-by-n'); end
    if size(B(t0)) ~= [n m], error('B must be n-by-m'); end
    if size(R(t0)) ~= [m m], error('R must be m-by-m'); end

    [ts,Ss] = matrixODE(@ode45,...
                        @(t,S) -A(t)'*S-S*A(t)-Q(t)+S*B(t)*inv(R(t))*B(t)'*S,...
                        fliplr(tspan),Qf);
end