function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 

nb = size(X0,1);
X = zeros(size(X0));
al = size(X);
for i = 1 : nb
    
    X_tp = X0(i,:);
    %aaa = size(X_temp)
    for k = 1 : 5
        X_tp = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), X_tp);
    end
    al = size(X_tp);
    X(i,:) = X_tp;
    
end


function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)

% projecting again
x1n = K * R1*( X0' - C1);
x1n = (x1n ./ repmat(x1n(3, :), [3, 1]))';
x2n = K * R2 * (X0' - repmat(C2, [1 size(X0,1)]));
x2n = (x2n ./ repmat(x2n(3, :), [3, 1]))';
x3n = K * R3 * (X0' - repmat(C3, [1 size(X0,1)]));
x3n = (x3n ./ repmat(x3n(3, :), [3, 1]))';

% computing change in x
d = [ x1(1), x1(2), x2(1), x2(2), x3(1), x3(2) ]'; 
fx = [ x1n(1), x1n(2), x2n(1), x2n(2), x3n(1), x3n(2) ]';

J =[ Jacobian_Triangulation(C1, R1, K, X0)',...
     Jacobian_Triangulation(C2, R2, K, X0)',...
     Jacobian_Triangulation(C3, R3, K, X0)' ]';

err = (x1(1)-x1n(1))^2 + (x1(2)-x1n(1))^2 + ...
        (x2(1)-x2n(1))^2 + (x2(2)-x2n(1))^2 + ...
        (x3(1)-x3n(1))^2 + (x3(2)-x3n(1))^2;

d_X = (J'*J)\(J'*(d-fx));

% X_update
X = X0 + d_X';

al = size(X0);
al = size(d_X);
al = size(X);
end


function J = Jacobian_Triangulation(C, R, K, X)

%pixel component
xj = K * R *( X' - C);
ui = xj(1);
vi = xj(2);
wi = xj(3);

% extracting intrinsic matrix component.
fd = K(1,1);
pxi = K(1,3);
pyi = K(2,3);

% computing the Jacobian
dui = [ fd*R(1,1)+pxi*R(3,1), fd*R(1,2)+pxi*R(3,2), fd*R(1,3)+pxi*R(3,3)];
dvi = [ fd*R(2,1)+pyi*R(3,1), fd*R(2,2)+pyi*R(3,2), fd*R(2,3)+pyi*R(3,3)];
dwi = [ R(3,1), R(3,2), R(3,3)];


J = [(wi*dui-ui*dwi)/(wi^2);(wi*dvi-vi*dwi)/(wi^2)];
   
end
end