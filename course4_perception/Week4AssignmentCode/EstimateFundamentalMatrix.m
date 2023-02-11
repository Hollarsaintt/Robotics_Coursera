function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2

[n,m] = size(x1);
D = zeros(n,9);

for i=1:n
    D(i,1)= x1(i,1)*x2(i,1);
    D(i,2)= x1(i,1)*x2(i,2);
    D(i,3)= x1(i,1);
    D(i,4)= x1(i,2)*x2(i,1);
    D(i,5)= x1(i,2)*x2(i,2);
    D(i,6)= x1(i,2);
    D(i,7)= x2(i,1);
    D(i,8)= x2(i,2);
    D(i,9)= 1;
end

[U, S, V] = svd(D);

hl = V(:,9);

Hp = [hl(1) hl(2) hl(3);hl(4) hl(5) hl(6);hl(7) hl(8) hl(9)];

[U2,S2,V2] = svd(Hp);

S2(3,3)=0;

Fb = U2 * S2 * V2.';

F = Fb/norm(Fb);

end