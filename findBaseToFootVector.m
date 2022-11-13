function [foot_position] = findBaseToFootVector(q, r_b_hip_yaw, leg_dimensions, distance_hip_joints)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% leg dimensions
upper_leg = leg_dimensions(1);
lower_leg = leg_dimensions(2);

% Homogenous Transformation from B frame to Hip yaw joint frame
T_B_hip_yaw = jointToTransformBHipYaw(q, r_b_hip_yaw);

% Homogeneous transfomation from hip yaw joint frame to hip pitch frame
r_hip_yaw_hip_pitch = [distance_hip_joints; 0; 0];
T_hip_yaw_hip_pitch = jointToTransformHipYawToHipPitch(q, r_hip_yaw_hip_pitch);

% Homogeneous tranformation from hip pitch joint frame to knee pitch frame
r_hip_pitch_knee_pitch = [upper_leg; 0; 0];
T_hip_pitch_knee_pitch = jointToTransformHipPitchKneePitch(q, r_hip_pitch_knee_pitch);

% homogeneous transformation from knee pitch joint frame to foot frame
r_knee_pitch_foot = [lower_leg; 0; 0];
T_knee_pitch_foot = jointToTransformKneePitchFoot(q, r_knee_pitch_foot);

% Homogeneous transformation from B to Q = [FL, BL, FR, BR]
T_BQ = T_B_hip_yaw * T_hip_yaw_hip_pitch * T_hip_pitch_knee_pitch * T_knee_pitch_foot;

% End effector position: From base frame to foot frame Q = [FL, FR, BL, BR]
foot_position = T_BQ(1:3, 4);
end

