function [T_knee_pitch_foot] = jointToTransformKneePitchFoot(q,r_knee_pitch_foot)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% % For the beginning we assume the foot frame orientation to be the same
% % as the knee_pitch frame.
foot_euler_angles = q(4:6)';
C_knee_pitch_foot = eul2rotm(foot_euler_angles);

% C_knee_pitch_foot = eye(3);
T_knee_pitch_foot = [
    C_knee_pitch_foot r_knee_pitch_foot;
    zeros(1,3) 1;
];
end

