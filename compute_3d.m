function [points] = compute_3d( camera, disparity, u, v)
%compute_3d Compute the 3d position of pixels. See also
%       read_disparity_image, get_point_cloud
%
%    [points] = compute_3d(camera, disparity, u, v)
%       Computes the 3D locations of selected pixels in the disparity image.
%       
%       camera: camera parameters.
%       disparity: disparity image
%       u: (1xN array) x position of the pixels to be transformed to 3D
%       v: (1xN array) y position of the pixels to be transformed to 3D
%
%       Example: 
%       imshow(disparity);
%       [u, v] = getpts();
%       points = compute_3d(camera, disparity, round(u)', round(v)');
%       plot3(points(1,:), points(2,:),points(3,:), '.');
%

disparity_points = disparity(sub2ind(size(disparity), v,u));
disparity_points = max(0, disparity_points);

%compute the location of all points with respect to the camera.
zCamt = (camera.intrinsic.fx * camera.extrinsic.baseline) ./ disparity_points;
xCamt = -(zCamt / camera.intrinsic.fx) .* (camera.intrinsic.u0 - u);
yCamt = -(zCamt / camera.intrinsic.fy) .* (camera.intrinsic.v0 - v);

% Transform the points from the camera point of view to the vehicle ref frame.
% Output points are in vehicle reference frame
P = [0 0 1 0; 
     -1 0 0 0; 
     0 -1 0 0;
     0 0 0 1];
rotmat = [cos(camera.extrinsic.pitch) 0  sin(camera.extrinsic.pitch)
       0 1 0 
       -sin(camera.extrinsic.pitch) 0 cos(camera.extrinsic.pitch)];
offset = [camera.extrinsic.x; 
          camera.extrinsic.y; 
          camera.extrinsic.z];
Textrinsic = [rotmat, offset; zeros(1,3), 1] * P; %camera to vehicle coordinate transfomration
points =  Textrinsic* [xCamt;yCamt;zCamt;ones(1,length(zCamt))];% rotmat * v + offset;
points = points(1:3,:);
end