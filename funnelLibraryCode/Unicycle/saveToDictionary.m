%% Script for saving it into dictionary format from raw files: saveToDictionary

%% Add directories
addpath('./lib/');

%% Load the precomputed files

load('./precomputedData/nominalTrajectory.mat');
load('./precomputedData/LQRGainsAndCostMatrices.mat');
load('./precomputedData/setInvarianceCertificates.mat');
load('./precomputedData/metaData.mat'); %has initialState and finalState

N = length(time_instances);
n = size(x_nom,1); %dimensionality of state-space

funnel = struct('time',NaN(1,N),'trajectory',NaN(n,N),'RofA',NaN(n,n,N));

funnel.time = time_instances;
funnel.trajectory = x_nom;
for k=1:N
    funnel.RofA(:,:,k) = ellipsoidMatrices(:,:,k)/rhoScaling(k);

    M = ellipsoidMatrices(:,:,k)/rhoScaling(k);
    M_nd = project_ellipsoid_matrix_nD(M, [1 2 3]);

end

funnelLibrary(num2str([finalState(1) finalState(2)])) = funnel;


%for projections down to 6D space

% function plotFunnel_3D(x_nom, ellipsoidMatrix, rhoScaling, projectionDims)
%     figure; view(3);
%     hold on; grid on; axis equal;
% 
%     P = plottingFnsClass();
% 
%     if nargin < 4
%         projectionDims = [1 2 3]; %if not specified, by default x-y-z projection
%     end
% 
%     %plot ellipsoidal invariant sets in 3D
%     for k=1:1:size(x_nom,2)
%         M = ellipsoidMatrix(:,:,k)/rhoScaling(k);
%         M_xyz = P.project_ellipsoid_matrix_3D(M, projectionDims);
%         center = [x_nom(projectionDims(1),k), x_nom(projectionDims(2),k), x_nom(projectionDims(3),k)]';
%         P.plotEllipsoid(center, M_xyz);
%     end 
% 
%     outlet = ellipsoidMatrix(:,:,end)/rhoScaling(end);
%     outlet_xyz = P.project_ellipsoid_matrix_3D(outlet, projectionDims);
%     center = [x_nom(projectionDims(1),end), x_nom(projectionDims(2),end), x_nom(projectionDims(3),end)]';
%     P.plotEllipsoid(center, outlet_xyz, 'red');
% 
%     %nominal trajectory
%     plot3(x_nom(projectionDims(1),:),x_nom(projectionDims(2),:),x_nom(projectionDims(3),:),'--b');
% 
%     %formatting
%     title('Invariant Ellipsoidal Sets along the nominal trajectory');
%     %xlabel('p_x');    ylabel('p_y');    zlabel('p_z');
%     xlabel(['x_{', num2str(projectionDims(1)), '}'])
%     ylabel(['x_{', num2str(projectionDims(2)), '}'])
%     zlabel(['x_{', num2str(projectionDims(3)), '}'])
% end

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