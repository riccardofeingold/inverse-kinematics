function [T_B_hip_yaw] = jointToTransformBHipYaw(q, r_b_hip_yaw)
%JOINTTOTRANSFORMTBHIPYAW Summary of this function goes here
%   Detailed explanation goes here
hip_yaw = q(1);
r_b_hip_yaw
angle = atan2(r_b_hip_yaw(2), r_b_hip_yaw(1));

C_B_hip_yaw = [
    cos(angle + hip_yaw) -sin(angle + hip_yaw)  0;
    sin(angle + hip_yaw)  cos(angle + hip_yaw)  0;
    0             0             1;
];
T_B_hip_yaw = [
    C_B_hip_yaw r_b_hip_yaw;
    zeros(1,3)  1;
];
end

