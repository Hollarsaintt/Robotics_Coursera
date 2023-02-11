function [rads1,rads2] = computeRrInverseKinematics(X,Y)

X = 0; Y = 1.5;
syms theta1 theta2 real ;

% % my code
% rads1= acos(X);
% 
% d = atan2(Y,X) - rads1;
% 
% k = (2*(sin(d))^2) - 1;
% rads2= pi - acos(k);

%my code
a = sqrt(2*(1 - cos(pi-theta2)));
d = asin(sin(pi-theta2)/a); 

xEpp = a * cos (d + theta1) == X;
yEpp = a * sin(d + theta1) == Y;

sol = solve([xEpp, yEpp], [theta1, theta2]);
rads1 = sol.theta1;
rads2 = sol.theta2;

end