function [T_hip_pitch_knee_pitch] = jointToTransformHipPitchKneePitch(q,r_hip_pitch_knee_pitch)
%JOINTTOTRANSFORMHIPPITCHKNEEPITCH Summary of this function goes here
%   Detailed explanation goes here
knee_pitch = q(3);
C_hip_pitch_knee_pitch = [
     cos(knee_pitch)  0 sin(knee_pitch);
     0                1 0;
    -sin(knee_pitch)  0 cos(knee_pitch);
];
T_hip_pitch_knee_pitch = [
    C_hip_pitch_knee_pitch r_hip_pitch_knee_pitch;
    zeros(1,3) 1;
];
end

