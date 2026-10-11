clc; clearvars; close all;

%param values imported from the experiments running script
envLB = 0;
envUB = 50;
obstacleSizeRange = [1 3]; %radius of circular obstacles

%input the number of obstacles
numObstaclesArray = 0:5:120;
numRandomSeeds = 100; %randomized environments generation (for better accuracy)

obstacleDensities = NaN(1,numel(numObstaclesArray));
for i=1:numel(numObstaclesArray)
    numObstacles = numObstaclesArray(i);
    
    density = NaN(1,numRandomSeeds);
    for j=1:numRandomSeeds %random environments
        randomObstacles = addRandomObstacles(numObstacles, envLB, envUB, obstacleSizeRange);
        density(j) = obstacleDensity(randomObstacles, envLB, envUB) * 100;  % defaults: res = 0.05 (seems "convergent" enough)
    end

    obstacleDensities(i) = mean(density,"omitnan");
end

numObstaclesArray'
obstacleDensities'

% figure; hold on; axis equal; 
% xlim([envLB, envUB]); ylim([envLB, envUB]);
% drawAllObstacles(randomObstacles);

%% Function definitions

function randomObstacles = addRandomObstacles(numObstacles, envLB, envUB, obstacleSizeRange)
    
    randomObstacles = NaN(numObstacles,3);

    for count=1:numObstacles 
        
        location = rand(1,2).*(envUB - envLB) + envLB;
        size = obstacleSizeRange(1) + (obstacleSizeRange(2) - obstacleSizeRange(1))*rand();

        randomObstacles(count,:) = [location size];
    end
end

function [density, occ] = obstacleDensity(obstacles,envLB,envUB,res)
%OBSTACLEDENSITY Fraction of the square [envLB,envUB]x[envLB,envUB] covered by circular obstacles.
%
%   density = obstacleDensity(obstacles,envLB,envUB)
%   density = obstacleDensity(obstacles,envLB,envUB,res)
%   [density, occ] = obstacleDensity(obstacles,envLB,envUB,res)
%
%   obs : numObstacles x 3 array, rows = [x, y, radius]
%   res : grid cell size (default 0.05 -> 1000x1000 grid for L = 50)
%
%   density : covered area / L^2, in [0,1]. Overlaps are counted once and
%             any part of a circle outside the square is ignored.
%   occ     : logical occupancy grid (rows = y, cols = x), cell centers at
%             res/2 : res : L. Reusable for planning / collision checks.
%
%   Accuracy: error scales with res. Halve res once to confirm convergence.

    L = envUB - envLB; %side length of the square workspace 
    if nargin < 4 || isempty(res), res = 0.05; end

    if ~isempty(obstacles) && size(obstacles, 2) ~= 3
        error('obs must be numObstacles x 3: [x, y, radius].');
    end

    n = round(L / res);
    g = ((1:n) - 0.5) * res;          % cell-center coordinates
    occ = false(n, n);

    for i = 1:size(obstacles, 1)
        cx = obstacles(i,1); cy = obstacles(i,2); r = obstacles(i,3);

        % Only touch cells inside the circle's bounding box (clipped to domain)
        ix = max(1, floor((cx - r)/res) + 1) : min(n, ceil((cx + r)/res));
        iy = max(1, floor((cy - r)/res) + 1) : min(n, ceil((cy + r)/res));
        if isempty(ix) || isempty(iy), continue; end   % fully outside

        dx2 = (g(ix) - cx).^2;        % 1 x nx
        dy2 = (g(iy).' - cy).^2;      % ny x 1
        occ(iy, ix) = occ(iy, ix) | (dy2 + dx2 <= r^2);
    end

    density = nnz(occ) / numel(occ);
end

%function to draw all obstacles
function drawAllObstacles(addedObstacles)            
    
    for i=1:size(addedObstacles,1)
        center = addedObstacles(i,1:2); %[x,y,r]
        radius = addedObstacles(i,3);
    
        th = 0:pi/50:2*pi;
        xunit = radius * cos(th) + center(1);
        yunit = radius * sin(th) + center(2);
        
        %plot(xunit, yunit,'k','LineWidth', 1.1);
        fill(xunit,yunit,[0 0 0])
    end
end

%% calculated values (for temp paper writing/plotting purposes)
% 0	0
% 5	2.66174900000000
% 10	5.01759100000000
% 15	7.53096900000000
% 20	9.99683700000000
% 25	12.2786410000000
% 30	14.4842430000000
% 35	16.7000670000000
% 40	18.5731850000000
% 45	20.9799110000000
% 50	23.1320490000000
% 55	25.1846840000000
% 60	27.2035030000000
% 65	28.7389760000000
% 70	30.9327250000000
% 75	32.3865760000000
% 80	34.1052560000000
% 85	36.0227800000000
% 90	37.7479800000000
% 95	39.2630140000000
% 100	40.9484660000000
% 105	42.2957630000000
% 110	43.6384950000000
% 115	44.7713660000000
% 120	46.7554660000000