
%particular type of polynomial representation compatible with spots
t = msspoly('t');
x = msspoly('x',n); 


%sim_pp2p converts piecewise Polynomial to msspoly

ui = sim_pp2p(t,upp); 
x0i = sim_pp2p(t,spline(taus,ppval(xpp,taus)));
Pi = sim_pp2p(t,Ppp);
rhoi = sim_pp2p(t,rhopp);

ts = flipud(taus); %flip up down

%cell is an array but each entry can contain any type of datatypes
gammas = cell(1,N-1);
intervals = cell(1,N-1);
Ls = cell(1,N-1);
xdot = cell(1,N-1);
V    = cell(1,N-1);
rho  = cell(1,N-1);

for i = 1:N-1
    intervals{i} = [ts(i) ts(i+1)]-ts(i);
    Pii = reshape(Pi(:,i),n,n);

    xbari = (x-x0i(:,i));
    xdot{i} = f0(t,x,ui(:,i)-K(ts(i))*xbari);
    V{i} = xbari'*reshape(Pi(:,i),n,n)*xbari;
    rho{i} = rhoi(:,i);
end



k=2;
Vs = repmat(V{1},N-1,k);
rhos = repmat(V{1},N-1,k);
Vdots = repmat(V{1},N-1,k);
rhodots = repmat(V{1},N-1,k);

for i = 1:N-1
    tt = linspace(intervals{i}(1),intervals{i}(2),k);
    tts(i,:) = tt;
    Vs(i,:) = msubs(V{i},t,tt);
    Vdots(i,:) = msubs(diff(V{i},t)+diff(V{i},x)*xdot{i},t,tt);
    rhos(i,:)  = msubs(rho{i},t,tt);
    rhodots(i,:) = msubs(diff(rho{i},t),t,tt);
end

M = length(Vs(:));
for i =1:M
    warning('off','MATLAB:nearlySingularMatrix');
    tstart=now;
    [gammas{i},Ls{i}] = tv_poly_rho_L_samp(t,x,tts(i),rhos(i), ...
                                           rhodots(i),Vs(i),Vdots(i));
    if mod(i,10) == 0
        fprintf([ int2str(i) '/' int2str(M) '\n']);
    end
    timing{i} = now-tstart;
end

% NCORE=4;
% parpool('open',NCORE)
% M = length(Vs(:));
% parfor i =1:M
%    warning('off','MATLAB:nearlySingularMatrix');
%    tstart=now;
%    [gammas{i},Ls{i}] = tv_poly_rho_L_samp(t,x,tts(i),rhos(i), ...
%                                           rhodots(i),Vs(i),Vdots(i));
%    if mod(i,10) == 0
%        fprintf([ int2str(i) '/' int2str(M) '\n']);
%    end
%    timing{i} = now-tstart;
% end
% parpool close