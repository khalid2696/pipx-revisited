%% Mohamed Khalid M Jaffar (July '24)
% 

%% First get the funnel into a structure format
funnel = struct('time',NaN(1,N),'trajectory',NaN(6,N),'RofA',NaN(6,6,N));

%Saving the data
for j = 1:1:N-1
    %Pi comes from Lagrange multiplier.m - substitute t in all polynomials with 0 
    St = double(reshape(subs(Pi(:,j),t,0),n,n)); %reshape into n-by-n matrix (n - dimension of system)
    
    %x0i comes from Lagrange multiplier.m - substitute t in all polynomials with 0
    xt = double(subs(x0i(:,j),t,0));
    
    ellipsoid = St./ppval(rhopp,ts(j));
    
    funnel.time(j) = ts(j);
    funnel.trajectory(:,j) = xt;
    funnel.RofA(:,:,j) = ellipsoid;
end

funnel.time(N) = ts(N);
funnel.trajectory(:,N) = x0s(end,:); %size of x0s is not equal to N, so use 'end'
funnel.RofA(:,:,N) = S0;

%% Now save it into the dictionary
funnelLibrary(num2str([x_initial, y_initial])) = funnel;