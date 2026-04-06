clc;
close all;

%filepath = "C:\Users\henry\OneDrive\Documents\GitHub\ENGR498-GUI-Project\ENGR-498-Project\assets\scan_002\processed\filtered\cloud_filtered.las";
%filepath = "C:\Users\henry\Downloads\LAW2_matched_filtered.las";
filepath = "C:\Users\henry\Downloads\slt3.las";

pc = lasFileReader(filepath);
ptCloud = readPointCloud(pc);
xyz = ptCloud.Location;

% -----------------------------
% Parameters
% -----------------------------
cellSize = 1.5;       % XY grid size in meters
groundMargin = 0.3;   % points within this height above local min are ground

x = xyz(:,1);
y = xyz(:,2);
z = xyz(:,3);

% Shift coordinates so indexing starts cleanly
x0 = min(x);
y0 = min(y);

ix = floor((x - x0) / cellSize) + 1;
iy = floor((y - y0) / cellSize) + 1;

nx = max(ix);
ny = max(iy);

% Store local min Z per cell
minZGrid = inf(nx, ny);

% Pass 1: compute local minimum Z in each cell
for k = 1:length(z)
    if z(k) < minZGrid(ix(k), iy(k))
        minZGrid(ix(k), iy(k)) = z(k);
    end
end

% Pass 2: classify points
groundIndex = false(size(z));
nonGroundIndex = false(size(z));

for k = 1:length(z)
    localMinZ = minZGrid(ix(k), iy(k));

    if z(k) <= localMinZ + groundMargin
        groundIndex(k) = true;
    else
        nonGroundIndex(k) = true;
    end
end

groundPoints = xyz(groundIndex, :);
nonGroundPoints = xyz(nonGroundIndex, :);

fprintf("Total points: %d\n", size(xyz,1));
fprintf("Ground points: %d\n", size(groundPoints,1));
fprintf("Non-ground points: %d\n", size(nonGroundPoints,1));

% -----------------------------
% Visualize
% -----------------------------
figure;
pcshow(groundPoints, repmat([0 1 0], size(groundPoints,1), 1)); % green
hold on;
pcshow(nonGroundPoints, repmat([1 0 0], size(nonGroundPoints,1), 1)); % red
title(sprintf('Grid-Based Ground Filter | cellSize=%.2f, margin=%.2f', cellSize, groundMargin));
view(60,25);