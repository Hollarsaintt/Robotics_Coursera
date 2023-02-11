function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 3) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 3) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points


nb = size(x1,1);

J1 = [R1,-R1*C1];
F1 = K*J1;
J2 = [R2,-R2*C2];
F2 = K*J2;



for i = 1 : nb
    px1 = cmatrix([x1(i,:),1]);
    r1 = px1*F1;
   
    px2 = cmatrix([x2(i,:),1]);
    r2= px2*F2;
    
    B = [r1;r2]; 
    
    [~,~,V] = svd(B);
    
    X(i,:) = (V(1:3,4)/V(4,4))';
end

end

