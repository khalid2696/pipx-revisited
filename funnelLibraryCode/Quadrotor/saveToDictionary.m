%% Script for saving it into dictionary format from raw files: saveToDictionary

if ~exist('funnelLibrary', 'var')
    funnelLibrary = dictionary();
end

%% Add directories
addpath('./lib/');

%% Load the precomputed files

load('./precomputedData/nominalTrajectory.mat');
load('./precomputedData/LQRGainsAndCostMatrices.mat');
load('./precomputedData/setInvarianceCertificates.mat');
load('./precomputedData/metaData.mat'); %has initialState and finalState

N = length(time_instances);
projectionDimensions = [1 2 3 4 5 6];

n_x = size(x_nom,1); %dimensionality of state space
n_u = size(u_nom,1); %dimensionality of input space

m = length(projectionDimensions); %hacky: just for now (ideally m should be equal to n_x!)

funnel = struct('time',NaN(1,N), 'trajectory', NaN(m,N), 'trajectory_complete',NaN(n_x,N), ...
                'RofA',NaN(m,m,N), 'RofA_complete', NaN(n_x,n_x,N), ...
                'nominalControl', NaN(n_u,N), 'feedbackControlGains', NaN(n_u,n_x,N));

%save the time
funnel.time = time_instances;

%save the nominal trajectory
funnel.trajectory_complete = x_nom;
funnel.nominalControl = u_nom;
funnel.feedbackControlGains = K;

x_nom_nd = NaN(m,N);
for i=1:length(projectionDimensions)
    x_nom_nd(i,:) = x_nom(projectionDimensions(i),:);
end

funnel.trajectory = x_nom_nd;

%save the ellipsoidal certificates of invariance
for k=1:N
    
    M = ellipsoidMatrices(:,:,k)/rhoScaling(k);
    funnel.RofA_complete(:,:,k) = M;  

    M_nd = project_ellipsoid_matrix_nD(M, projectionDimensions);
    funnel.RofA(:,:,k) = M_nd;    
end

funnelLibrary(num2str([finalState(1) finalState(2)])) = funnel;


%for projections down to 6D space (hacky, temporary for now)
function M_nd = project_ellipsoid_matrix_nD(M, projection_dims)
    % Input:
    % M: nxn matrix defining the n-dimensional ellipsoid x^T M x < 1
    % projection_dims: m-element vector specifying which dimensions to project onto
    %                  (e.g., [1 2 3] for 3D xyz-plane, [1 3] for 2D xz-plane)

    n = size(M, 1); %get the dimensionality of matrix M
    m = length(projection_dims);

    basisMatrix = zeros(n,m);

    for i = 1:m
        basisMatrix(projection_dims(i),i) = 1;
        basisMatrix(projection_dims(i),i) = 1;
        basisMatrix(projection_dims(i),i) = 1;
    end

    M_nd = inv(basisMatrix' *inv(M) * basisMatrix);
end