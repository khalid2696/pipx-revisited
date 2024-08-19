function S = inner_ellipse(P,x0,xT)
    n = size(x0,1);
    prog = mssprog;
    [prog,S] = new(prog,n,'psd');
    [prog,Q] = new(prog,n+1,'psd');
    [prog,t] = new(prog,1,'pos');
    
    prog.eq = Q - [ t*(1-x0'*P*x0) - (1-xT'*S*xT)  x0'*P-xT'*S ; ...
                    P*x0-S*xT                      S-t*P];
    
    prog.sedumi = trace(S);
    S = double(prog(S));
end
