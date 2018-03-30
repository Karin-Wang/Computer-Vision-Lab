clear all;
close all;
clc;

%% Setting up 
subfolder = @(base, sub) [base '/' sub];
merge_file = @(dir) subfolder(dir.folder, dir.name);

img_folder = 'urban/left';
disp_folder = 'urban/disp_gray';

img_files = dir(img_folder);
img_files = img_files(3:end);
disp_files = dir(disp_folder);
disp_files = disp_files(3:end);

for k = 1:numel(disp_files)
    disp_files(k).folder = disp_folder;
    img_files(k).folder = img_folder;
end

%% Load dataset
load('camera');
load('bundle_adjustement_data.mat');
% Contains ba_data struct:
% ba_data.points3d: 3d datapoints created at time = 1;
% ba_data.nr_poses: number of camera poses after the first frame (t=1)
% for i = 1:ba_data.nr_poses
%   ba_data.frame(i).image_index:           image index of image directory
%   ba_data.frame(i).indices_features:      feature indices to 3d points 
%   ba_data.frame(i).feature_locations_px 	pixel locations of features (2d)
%   ba_data.frame(i).pose                   estimate of the pose of frame i in world frame
% end

%% Tip 1: Start with running only on subset of the data by setting:
%  ba_data.nr_poses = 2; %<-- Uncomment this to run on smaller trajectory

%% Write funciton that maps ba_data.points3d to image plane

% Question 2a
% Write function that maps the 3d poitns to the image of pose i: [u,v] = map2image(P,p,camera)

%% Compute total reprojection error for initial X

% Get initial X
% X = ....
% ....
% ....

% Question 2b
rmse0 = optimize_reprojection_error(X, data, camera)

%% Run Bundle Adustment 

%Question 2c
[rmse0,rmse1,X] = optimize_reprojection_error(X, data, camera);

%% Plot Results