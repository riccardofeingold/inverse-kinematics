function [foot_position] = findBaseToFootVector(q, body_dimensions, leg_dimensions, distance_hip_joints)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% joint angles
hip_yaw = q(1);
hip_pitch = q(2);
knee_pitch = q(3);

% body dimensions
body_length = body_dimensions(1);
body_width = body_dimensions(2);
base_frame_height = body_dimensions(3);

% leg dimensions
upper_leg = leg_dimensions(1);
lower_leg = leg_dimensions(2);

% Homogenous Transformation from B frame to Hip yaw joint frame
C_B_hip_yaw = [
    cos(hip_yaw) -sin(hip_yaw)  0;
    sin(hip_yaw)  cos(hip_yaw)  0;
    0             0             1;
];
r_b_hip_yaw = [body_width/2; body_width/2; 0];
T_B_hip_yaw = [
    C_B_hip_yaw r_b_hip_yaw;
    zeros(1,3)  1;
];

% Homogeneous transfomation from hip yaw joint frame to hip pitch frame
C_hip_yaw_hip_pitch = [
     cos(hip_pitch)  0 sin(hip_pitch);
     0               1 0;
    -sin(hip_pitch)  0 cos(hip_pitch);
];
r_hip_yaw_hip_pitch = [distance_hip_joints; 0; 0];
T_hip_yaw_hip_pitch = [
    C_hip_yaw_hip_pitch r_hip_yaw_hip_pitch;
    zeros(1,3) 1;
];

% Homogeneous tranformation from hip pitch joint frame to knee pitch frame
C_hip_pitch_knee_pitch = [
     cos(knee_pitch)  0 sin(knee_pitch);
     0                1 0;
    -sin(knee_pitch)  0 cos(knee_pitch);
];
r_hip_pitch_knee_pitch = [upper_leg; 0; 0];
T_hip_pitch_knee_pitch = [
    C_hip_pitch_knee_pitch r_hip_pitch_knee_pitch;
    zeros(1,3) 1;
];

% homogeneous transformation from knee pitch joint frame to foot frame
% % For the beginning we assume the foot frame orientation to be the same
% % as the knee_pitch frame.
C_knee_pitch_foot = eye(3);
r_knee_pitch_foot = [lower_leg; 0; 0];
T_knee_pitch_foot = [
    C_knee_pitch_foot r_knee_pitch_foot;
    zeros(1,3) 1;
];

T_BQ = T_B_hip_yaw * T_hip_yaw_hip_pitch * T_hip_pitch_knee_pitch * T_knee_pitch_foot;

% End effector position: From base frame to foot frame Q = [FL, FR, BL, BR]
foot_position = T_BQ(1:3, 4);
end

