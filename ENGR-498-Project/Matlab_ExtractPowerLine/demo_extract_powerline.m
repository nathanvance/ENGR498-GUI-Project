% /****************************************************************************
%  * Please cite the following paper, If you use this code in your work.
%  *
%  * Zhenwei Shi, Yi Lin, and Hui Li. "Extraction of urban power lines and 
%  * potential hazard analysis from mobile laser scanning point clouds." 
%  * International Journal of Remote Sensing 41, no. 9 (2020): 3411-3428.
%  *
%  * The paper can be downloaded from
%  * https://www.tandfonline.com/doi/full/10.1080/01431161.2019.1701726
%  * Copyright
%  * Institute of Remote Sensing & GIS, 
%  * School of Earth and Space Sciences, Peking University (www.pku.edu.cn)
%  * Zhenwei Shi; Yi Lin; Hui Li
%  * contact us: zwshi@pku.edu.cn; lihui@pku.edu.cn
% *****************************************************************************/

% Extracting powerline from mobile lidar point cloud
% function: [isPLIndex] = extractPLs(pointcloud,radius,angleThr,LThr)
% pointcloud: mobile lidar point cloud [x y z]
% radius: Neighborhood point search radius
% angleThr: Threshold value of the angle between normal vector and (0 0 1)
% LThr: linear feature L satisfy the conditions L≥LThr for power line
% point.

% Example
% radius = 0.5;
% angleThr = 10;
% LThr = 0.98;
% [isPLIndex] = extractPLs(pointcloud,radius,angleThr,LThr);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Function wrapper for use in python
function [PL, poly, groundPoints] = demo_extract_powerline(filepath)

%% step1 Mobile LiDAR filtering (Single LAS file)
clc;
close all;
tic

% Load your LAS file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%lasFile = 'powerlineAerialLidarData.las';
%pc = lasFileReader(lasFile);
pc = lasFileReader(filepath);
ptCloud = readPointCloud(pc);

% Extract Nx3 matrix
xyz = ptCloud.Location;

% Histogram ground estimation (same method as original code)
[counts,centers] = hist(xyz(:,3),50);
[~,I] = max(counts);

% Remove ground by threshold
nonGroundIndex = xyz(:,3) > centers(I+3);

% Save points
nonGroundPoints = xyz(nonGroundIndex, :);

% Ground index — the opposite of non-ground
groundIndex = xyz(:,3) <= centers(I+3);

% Extract ground points (THIS is what you need)
groundPoints = xyz(groundIndex, :);

toc

figure
pcshow(nonGroundPoints)
title('Non ground points')
view(60,25)

%print(gcf,'-dpng','-r300', 'f2_candidate powerline points.png')

%% step1.5 Extract powerline candidate points (correct function call)

% Parameters from original code
%radius   = 0.5;
%angleThr = 10;
%LThr     = 0.98;

%modified parameters from Claude
radius   = 0.5;
angleThr = 10;
LThr     = 0.90;

% Make sure extractPLs.m is available
if exist('extractPLs','file') ~= 2
    error('extractPLs.m not found. Add 3DLiDAR ExtractPowerLine folder to your MATLAB path.');
end

% Run powerline extraction exactly as original code expects
isPLIndex = extractPLs(nonGroundPoints, radius, angleThr, LThr);

% Convert to logical (necessary)
isPLIndex = logical(isPLIndex);

% Safety check
if isempty(isPLIndex) || numel(isPLIndex) ~= size(nonGroundPoints,1)
    error('extractPLs returned invalid index mask. The LAS point format must match expected input.');
end

% Visualize
figure
pcshow(nonGroundPoints(isPLIndex,:))
title('Candidate Powerline Points')
view(60,25)
print(gcf,'-dpng','-r300', 'f2_candidate powerline points.png')

%% step2 Euclidean clustering
ptCloud = pointCloud(nonGroundPoints(isPLIndex,:));
minDistance = 2.0;
[labels,numClusters] = pcsegdist(ptCloud,minDistance);

figure
pcshow(ptCloud.Location,labels)
title('Candidate power line clusters')
view(60,25)
print(gcf,'-dpng','-r300', 'f3_candidate powerline points clusters.png')

index = ones(size(labels,1),1);
power_lines = [];
for i=1:numClusters
    cluster = index*i == labels;
    if(sum(cluster)<15)
       continue; 
    end    
    xyzs = ptCloud.Location(cluster,:);
    power_lines = [power_lines; xyzs];
end
PLs = pointCloud(power_lines);
figure
pcshow(PLs);
title('Power line clusters')
view(60,25)
print(gcf,'-dpng','-r300', 'f4_powerline points clusters.png')


%% step3 Power line modeling
ptCloud = PLs;
minDistance = 0.3;
[labels,numClusters] = pcsegdist(ptCloud,minDistance);
figure
show_segs(ptCloud.Location,labels,1);
title('Different clusters are given different colors')
view(60,25)
print(gcf,'-dpng','-r300', 'f5_colorization clusters.png')

counts = [];
index = ones(size(labels,1),1);

% Constructing structure based on Clustering
for i=1:numClusters
    %cluster_index = index*i == labels;
    %replaced the top code with the bottom line
    cluster_index = (labels == i);
    cluster_raw = ptCloud.Location(cluster_index,:); 
    xyzs_new = cluster_raw;
    powerLines(i).Location = xyzs_new;
    powerLines(i).Label = 0;
    powerLines(i).Count = size(xyzs_new,1);
    powerLines(i).Ids = [];
    counts = [counts; powerLines(i).Count];
end

% Get the sorting ID of cluster points
[counts_new, ind] = sort(counts,'descend');
[powerLines_pro ind]= merge(powerLines, ind);
[powerLines_pro ind]= merge(powerLines_pro, ind);
[powerLines_pro ind]= merge(powerLines_pro, ind);

powerLines_new = [];
colors = [];
for i = 1:size(powerLines_pro,2)
    powerLines_new = [powerLines_new; powerLines_pro(i).Location];
    colors = [colors; repmat(rand(1,3),size(powerLines_pro(i).Location,1),1)];
end
% ptpl = pointCloud(powerLines_new,'Color',colors)
figure
 % pcshow(ptpl)
pcshow(powerLines_new,colors)
title('Power line clusters')
view(60,25)
print(gcf,'-dpng','-r300', 'f6_powerLines clusters.png')

% Calculate the power line length and delete the power line less than the length threshold
i = 1;
while i <= size(powerLines_pro,2)
    dist = getDist(powerLines_pro(i).Location);
    if dist < 1.5
        powerLines_pro(i) = [];
    else
        i = i + 1;
    end
end

% Calculate the length of the power line
i = 1;
sumdist = 0;
while i <= size(powerLines_pro,2)
    dist = getDist(powerLines_pro(i).Location);
    i = i + 1;
    sumdist = sumdist + dist;
end
sumdist

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%NEW%%%%%%%%%%%%%%%
numWires = length(powerLines_pro);
polyResults(numWires) = struct('p',[],'S',[],'mu',[]);

for i = 1:numWires
    pts = powerLines_pro(i).Location;

    % choose wire direction (horizontal axis)
    x = pts(:,1);
    z = pts(:,3);   % height axis

    % fit quadratic catenary proxy
    [p,S,mu] = polyfit(x,z,2);

    polyResults(i).p = p;
    polyResults(i).S = S;
    polyResults(i).mu = mu;
end

for i = 1:length(polyResults)
    fprintf("\n=== Wire %d ===\n", i);

    % Coefficients
    fprintf("p = [%.6f  %.6f  %.6f]\n", polyResults(i).p);

    % Error stats
    fprintf("S.normr = %.6f | S.df = %d | S.rsquared = %.4f\n", ...
        polyResults(i).S.normr, polyResults(i).S.df, polyResults(i).S.rsquared);

    % Mean & scale
    fprintf("mu = [mean=%.6f, std=%.6f]\n", polyResults(i).mu(1), polyResults(i).mu(2));
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Insert points to sparse power lines
for i = 1:size(powerLines_pro,2)
    powerLines_pro_new(i).Location = insert_3D(powerLines_pro(i).Location, 0.1);
end
% save('C:\Users\Lily\Desktop\PLM','powerLines_pro_new')

% powerLines_pro_new = powerLines_pro;
% visualization
powerLines_new = [];
colors = [];
for i = 1:size(powerLines_pro_new,2)
    powerLines_new = [powerLines_new; powerLines_pro_new(i).Location];
    temp = repmat(rand(1,3),size(powerLines_pro_new(i).Location,1),1);
    colors = [colors; temp];
    PLMs(i).Location = powerLines_pro_new(i).Location;
    PLMs(i).Color = temp;
end
% ptplm = pointCloud(powerLines_new,'Color',colors)
figure
% pcshow(ptplm.Location(1:1:end,:),ptplm.Color(1:1:end,:))
pcshow(powerLines_new,colors)
title('Power line modeling')
% view(60,25)
% print(gcf,'-dpng','-r300', 'f7_Power line model.png')

locations = {powerLines_pro.Location};
labels    = {powerLines_pro.Label};
counts    = {powerLines_pro.Count};
ids       = {powerLines_pro.Ids};

powerLines_pro_out = struct( ...
    'Location', {locations}, ...
    'Label',    {labels}, ...
    'Count',    {counts}, ...
    'Ids',      {ids} ...
);

poly = cell(1, numel(polyResults));

for i = 1:numel(polyResults)

    % Original z-values for this wire
    z = powerLines_pro(i).Location(:,3);

    % Compute R^2 manually
    SSE = polyResults(i).S.normr^2;
    SST = sum((z - mean(z)).^2);
    R2  = 1 - (SSE / SST);

    poly{i} = struct( ...
        'p', polyResults(i).p, ...       % coefficients
        'normr', polyResults(i).S.normr, ...
        'df', polyResults(i).S.df, ...
        'rsq', R2, ...                   % computed R²
        'mu_mean', polyResults(i).mu(1), ...
        'mu_std',  polyResults(i).mu(2) ...
    );
end
    PL = powerLines_pro_out;     % struct array with Location, Label, Count
    %poly = polyResults;      % polyfit output struct
end