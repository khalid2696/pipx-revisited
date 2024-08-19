t0 = 0;
tf = 10;

c = 3;

t = linspace(t0,tf,100);

rho = exp(c*(tf-t)/(tf-t0));

figure;
plot(t,rho);
grid on