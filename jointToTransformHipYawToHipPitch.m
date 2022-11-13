function [T_hip_yaw_hip_pitch] = jointToTransformHipYawToHipPitch(q,r_hip_yaw_hip_pitch)
%JOINTTRANSFORMHIPYAWTOHIPPITCH Summary of this function goes here
%   Detailed explanation goes here
hip_pitch = q(2);

C_hip_yaw_hip_pitch = [
     cos(hip_pitch)  0 sin(hip_pitch);
     0               1 0;
    -sin(hip_pitch)  0 cos(hip_pitch);
];
T_hip_yaw_hip_pitch = [
    C_hip_yaw_hip_pitch r_hip_yaw_hip_pitch;
    zeros(1,3) 1;
];
end

