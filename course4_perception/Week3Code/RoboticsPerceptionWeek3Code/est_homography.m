function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
H = [];
J = [];
for i = 1:length(video_pts)
    Jv = video_pts(i,1);
    bv = video_pts(i,2);
    Jl = logo_pts(i,1);
    bl = logo_pts(i,2);
    JJ = [ -Jv, -bv, -1, 0, 0, 0, Jv*Jl, bv*Jl, Jl];
    Jb = [ 0, 0, 0, -Jv, -bv, -1, Jv*bl, bv*bl, bl];
    J = cat(1,J,JJ,Jb);
end

[U,S,V] = svd(J);
D = V(:,9);
D = reshape(D,3,3);
H = D';

end

