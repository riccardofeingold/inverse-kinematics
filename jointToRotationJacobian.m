function [B_Jr_Q] = jointToRotationJacobian(q, r_b_hip_yaw, relative_joint_vectors)
% relative joint vector positions
r_hip_yaw_hip_pitch = relative_joint_vectors(1:3, 1);
r_hip_pitch_knee_pitch = relative_joint_vectors(1:3, 2);
% r_knee_pitch_foot = relative_joint_vectors(1:3, 3);

% Compute homogeneous transformation matrices
T_B_hip_yaw = jointToTransformBHipYaw(q, r_b_hip_yaw);
T_hip_yaw_hip_pitch = jointToTransformHipYawToHipPitch(q, r_hip_yaw_hip_pitch);
T_hip_pitch_knee_pitch = jointToTransformHipPitchKneePitch(q, r_hip_pitch_knee_pitch);
% T_knee_pitch_foot = jointToTransformKneePitchFoot(q, r_knee_pitch_foot);

% Transformation matrix from B frame to joint
% % 1 = hip_yaw, 2 = hip_pitch, 3 = knee_pitch, 4 = foot
T_B1 = T_B_hip_yaw;
T_B2 = T_B1 * T_hip_yaw_hip_pitch;
T_B3 = T_B2 * T_hip_pitch_knee_pitch;
% T_B4 = T_B3 * T_knee_pitch_foot;

% Rotation matrices
R_B1 = T_B1(1:3,1:3);
R_B2 = T_B2(1:3,1:3);
R_B3 = T_B3(1:3,1:3);
% R_B4 = T_B4(1:3,1:3);

% directions of the rotation axis of joint n w.r.t. joint n-1
n_1 = [0 0 1]';
n_2 = [0 1 0]';
n_3 = [0 1 0]';

B_Jr_Q = [
  R_B1*n_1 R_B2*n_2 R_B3*n_3
];
end

