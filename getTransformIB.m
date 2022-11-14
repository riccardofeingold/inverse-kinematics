function [T_IB, T_BI] = getTransformIB(q, body_orientation, leg_dimensions, distance_hip_joints, hip_yaw_locations, stationary_feet)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
C_IB = eul2rotm(body_orientation);

foot_positions = zeros(3,3);
counter = 1;
for k=1:4
    if stationary_feet(k) == 0
        foot_positions(1:3, counter) = findBaseToFootVector(q(1:3, k), hip_yaw_locations(1:3, k), leg_dimensions, distance_hip_joints);
        counter = counter + 1;
    end
end

% Position of feet in contact
% v1 = foot_positions(1:3,1)
% v2 = foot_positions(1:3,2)
% v3 = foot_positions(1:3,3)

% relative vector from B to I frame
r_BI = 1/3 * (foot_positions(1:3, 1) + foot_positions(1:3, 2) + foot_positions(1:3, 3));

% calculate homogeneous transformation from I to B frame => find r_IB
T_BI = [
    C_IB' r_BI;
    zeros(1,3) 1;
];

T_IB = inv(T_BI);
end

