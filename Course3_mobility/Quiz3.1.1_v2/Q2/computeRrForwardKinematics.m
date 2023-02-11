function [elbow,endeff] = computeRrForwardKinematics(rads1,rads2)
%%GIVEN THE ANGLES OF THE MOTORS, return an array of arrays for the
%%position of the elbow [x1,y1], and endeffector [x2,y2]

%my code: kinecmatics computed on paper myself.
a = sqrt(2*(1-cos(pi-rads2)));
d = asin(sin(pi-rads2)/ a);

elbow = [cos(rads1),sin(rads1)];
endeff =[a*cos(d + rads1), a*sin(d+rads1)];
