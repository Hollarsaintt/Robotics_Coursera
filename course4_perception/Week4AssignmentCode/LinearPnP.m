function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly

Nb = size(X,1);
X = [X,ones(Nb,1)];
x = [x,ones(Nb,1)];
x_c = K\x';
x = x_c';
B = zeros(2*Nb,12);


for i = 1:Nb
    B(i*2-1,1:4) = -X(i,:);
    B(i*2-1,9:12) = x(i,1)*X(i,:);
    B(i*2,5:8) = -X(i,:);
    B(i*2,9:12) = x(i,2)*X(i,:);
end

[~,~,V] = svd(B);

Pk = reshape(V(:,12),4,3)';
Riw = Pk(:,1:3);
tl = Pk(:,4);

[U,D,V] = svd(Riw);

if det(U*V') >0
    R = U*V';
    tl = tl/D(1);
elseif det(U*V') <0
    R = -U*V';
    tl = -tl/D(1);
end

C = -(R')*tl;

end





