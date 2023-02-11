function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Oututs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography
if H(3,3)<0
    H = -H;
end

a1 = H(:,1);
a2 = H(:,2);
t = H(:,3);

a1_norm = norm(a1);
a2_norm = norm(a2);

a1 = a1/a1_norm;
a2 = a2/a2_norm;
t = t/a1_norm;

[U, ~, V] = svd([a1, a2, cross(a1, a2)]); % Sigular Value Decomposition,SVD

R = U*[1 0 0; 0 1 0; 0 0 det(U*V')]*V';

% YOUR CODE HERE: Project the points using the pose

numbr = size(render_points,1);
proj_points = (K*(R*render_points' + repmat(t, 1, numbr)))';
proj_points(:,1) = proj_points(:,1) ./ proj_points(:,3);
proj_points(:,2) = proj_points(:,2) ./ proj_points(:,3);
proj_points(:,3) = proj_points(:,3) ./ proj_points(:,3);
proj_points = proj_points(:,1:2);

end
