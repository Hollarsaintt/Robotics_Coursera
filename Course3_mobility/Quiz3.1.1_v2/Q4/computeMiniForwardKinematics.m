function [endeff] = computeMiniForwardKinematics(rads1,rads2)

% my code all equations gotten from paper
alpha = 0.5*(rads1 + rads2) + pi;
beta = 0.5 * abs(rads1 - rads2);

L1 = 1; L2 = 2;
r = sqrt((L2^2 - L1^2)/(1-2*(L1^2)*cos(pi-beta)));1


endeff = [r*cos(alpha), r * sin(alpha)];

end