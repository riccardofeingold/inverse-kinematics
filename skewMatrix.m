function [skew] = skewMatrix(vector)
%SKEWMATRIX Summary of this function goes here
%   Detailed explanation goes here
wx = vector(1);
wy = vector(2);
wz = vector(3);

skew = [
    0 -wz wy;
    wz 0 -wx;
   -wy wx 0;
];
end
