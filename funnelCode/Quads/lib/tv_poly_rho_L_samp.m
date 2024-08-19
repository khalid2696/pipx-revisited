function [gamma,L] = tv_poly_rho_L_samp(t,x,ti,rho,rhodot,V,Vdot)
    prog = mssprog;
    
    Lxmonom = monomials(x,0:deg(Vdot,x)-deg(V,x));
    [prog,l] = new(prog,length(Lxmonom),'free');
    L1 = l'*Lxmonom;

    gamma = msspoly('g',1);
    prog.free=gamma;

    prog.sos = -(Vdot-rhodot+L1*(V-rho)-gamma);

    prog = sedumi(prog,gamma,0);
    
    gamma = double(prog(gamma));
    L = prog([ L1]);
end
