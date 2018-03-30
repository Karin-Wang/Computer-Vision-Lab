clear all;
close all;
clc;

%% Setting up
subfolder = @(base, sub) [base '/' sub];
merge_file = @(dir) subfolder(dir.folder, dir.name);

img_folder = '/Users/Kaering/Documents/MATLAB/assignment_5_data/urban/left';
disp_folder = '/Users/Kaering/Documents/MATLAB/assignment_5_data/urban/disp_gray';

img_files = dir(img_folder);
img_files = img_files(3:end);
disp_files = dir(disp_folder);
disp_files = disp_files(3:end);

for k = 1:numel(disp_files)
    disp_files(k).folder = disp_folder;
    img_files(k).folder = img_folder;
end

%% Load dataset
load('camera')
load('ransac_data');
% contains a 1x99 struct with the name frames:
% frames(i).image_index: index to image img_files
% frames(i).features.prev: matched image features in image prev (k-1)
% frames(i).features.now:  matched image features in image now k

%% Robust egomotion estimation
% HINT: you could use your own function ... from last week assignment
% or you could use the function find_transform(..) in this repository. Example
% usage: [rot,trans] = find_transform(q,p)
% The function return optimal transform q = rot*p + trans;

Transformation = zeros(4,4,99); % Pre-define Transformation matrix
for k = 1:length(frames)
    % Get features now and features prev
    % Get 2d points from dataset
    unow = round(frames(k).features.now(1,:));
    vnow = round(frames(k).features.now(2,:));
    uprev = round(frames(k).features.prev(1,:));
    vprev = round(frames(k).features.prev(2,:));
    
    % Get 3d points from 2D
    disparity_prev = read_disparity_image(merge_file(disp_files(frames(k).image_index.prev)));
    disparity_now = read_disparity_image(merge_file(disp_files(frames(k).image_index.now)));
    points_prev = compute_3d(camera, disparity_prev, uprev, vprev);
    points_now = compute_3d(camera, disparity_now, unow, vnow);
    
    switch 'ransac'
        case 'all'
            %% Question 1a
            % Use find_transform
            [rot,trans] = find_transform(points_prev,points_now);
            Transformation(:,:,k) = [rot trans;zeros(1,3) 1]; % k
        case 'ransac'
            %% Question 1e
            k_round = 100;
            threas = 0.3;
            numRandom = 3;
            [m,n] = size(points_now);
            max_ninliners = 0;
            best_inliers = [];
            for i = 1:k_round
                inliers = [];
                n_inliers = 0;
                p = points_now(:,randperm(n,numRandom));
                %                disp(p);
                p1 = p(:,1);
                p2 = p(:,2);
                p3 = p(:,3);
                %                disp(p15
                norm_prime1 = dot((p1-p2),(p3-p2));
                norm_prime = norm_prime1/norm(norm_prime1);
                for j = 1:n
                    pp = points_now(:,j);
                    distance = norm((p1-pp)*norm_prime);
                    %                    disp(distance);
                    if distance <= threas
                        inliers = [inliers,j];
                        n_inliers = n_inliers + 1;
                    end
                end
                if max_ninliners < n_inliers
                    max_ninliners = n_inliers;
                    best_inliers = inliers;
                end
            end
            points_prev1 = points_prev(:, best_inliers);
            points_now1 = points_now(:, best_inliers);
            [rot,trans] = find_transform(points_prev1,points_now1);
            Transformation(:,:,k) = [rot trans;zeros(1,3) 1]; % k
    end
    
end

%% a
initial_point=[0 0 0 1]';
current_point=initial_point;
paths=zeros(4,100);
for i=1:100
    paths(:,i)=current_point;
    if(i~=100)
        current_point=Transformation(:,:,i)*current_point;
    end
end
% delete the 4th row
paths(4,:,:)=[];
% A blue path in the 3D point cloud of frame0000
disparity=read_disparity_image('/Users/Kaering/Documents/MATLAB/assignment_5_data/urban/disp_gray/frame0000.png');
image=imread('/Users/Kaering/Documents/MATLAB/assignment_5_data/urban/left/frame0000.png');
resPointCloud = get_pointcloud( camera, disparity, image);
figure
draw_path_in_pointcloud(camera, disparity,image,paths);
% %%
% image1=imread('/Users/Kaering/Documents/MATLAB/assignment_5_data/urban/left/frame0100.png');
% image2=imread('/Users/Kaering/Documents/MATLAB/assignment_5_data/urban/left/frame0099.png');
% % image(image1);
% 
% figure
% k = 99;
% unow = round(frames(k).features.now(1,:));
% vnow = round(frames(k).features.now(2,:));
% % subplot(1,2,2);
% imshow(image1);
% hold on;
% plot(unow(best_inliers),vnow(best_inliers),'bs');
% unow(best_inliers) = [];
% vnow(best_inliers) = [];
% plot(unow,vnow,'rs');
% 
% figure
% % subplot(1,2,1);
% uprev = round(frames(k).features.prev(1,:));
% vprev = round(frames(k).features.prev(2,:));
% imshow(image2);
% hold on;
% plot(uprev(best_inliers),vprev(best_inliers),'bs');
% uprev(best_inliers) = [];
% vprev(best_inliers) = [];
% plot(uprev,vprev,'rs');
