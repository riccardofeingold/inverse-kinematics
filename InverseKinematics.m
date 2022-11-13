% Robot Dimensions
% % Body dimensions
body_length = 1;
body_width = 1;
base_frame_height = 0.5;

body_dimensions = [body_length, body_width, base_frame_height];

% % Leg dimensions
upper_leg = 2;
lower_leg = 2;

leg_dimensions = [upper_leg, lower_leg];

% Joint offsets
% % distance_hip_joints describes the distance between the hip_yaw joint
% % and the hip_pitch joint
distance_hip_joints = 0; 

% location of hip_yaw=1 joint relative to B frame
loc_FR1 = [body_length/2; -body_width/2; 0];
loc_FL1 = [body_length/2; body_width/2; 0];
loc_BR1 = [-body_length/2; -body_width/2; 0];
loc_BL1 = [-body_length/2; body_width/2; 0];

hip_yaw_location = [loc_FL1 loc_BL1 loc_FR1 loc_BR1];

% Joint variables
% % leg joints
% % Order: hip_yaw, hip_pitch, knee_pitch
q_FL = [0, 0, pi/2]';
q_FR = [0, 0, pi/2]';
q_BL = [0, 0, pi/2]';
q_BR = [0, 0, pi/2]';

% % feet joints
q_feet_FL = [0 0 0]';
q_feet_FR = [0 0 0]';
q_feet_BL = [0 0 0]';
q_feet_BR = [0 0 0]';

q = [
    q_FL      q_BL      q_FR      q_BR;
    q_feet_FL q_feet_BL q_feet_FR q_feet_BR;
];

% Which feet are not going to move
% 1 = moves; 0 = not moving
% stationary_feet = [FL, BL, FR, BR]
stationary_feet = [0, 0, 1, 0];

% Body Orientation (measured e.g. by an IMU)
body_roll = pi/10; % around x-axis
body_pitch = pi/5; % around y-axis
body_yaw = pi/20; % around z-axis

body_orientation = [body_roll, body_pitch, body_yaw];

[T_IB, T_BI] = getTransformIB(q, body_orientation, leg_dimensions, distance_hip_joints, hip_yaw_location, stationary_feet);
r_BI = T_BI(1:3,4);
r_IB = T_IB(1:3, 4);

% relative vectors from one joint frame to the next 
r_hip_yaw_hip_pitch = [distance_hip_joints; 0; 0];
r_hip_pitch_knee_pitch = [upper_leg; 0; 0];
r_knee_pitch_foot = [lower_leg; 0; 0];

relative_joint_vectors = [r_hip_yaw_hip_pitch r_hip_pitch_knee_pitch r_knee_pitch_foot];

% Calculate positional Jacobians
B_Jp_FL = jointToPositionJacobian(q(1:6, 1), hip_yaw_location(1:3, 1), relative_joint_vectors);
B_Jp_BL = jointToPositionJacobian(q(1:6, 2), hip_yaw_location(1:3, 2), relative_joint_vectors);
B_Jp_FR = jointToPositionJacobian(q(1:6, 3), hip_yaw_location(1:3, 3), relative_joint_vectors);
B_Jp_BR = jointToPositionJacobian(q(1:6, 4), hip_yaw_location(1:3, 4), relative_joint_vectors);

% % Calculate rotational Jacobians
% B_Jr_FL = jointToRotationJacobian(q, hip_yaw_location(1), relative_joint_vectors);
% B_Jr_BL = jointToRotationJacobian(q, hip_yaw_location(2), relative_joint_vectors);
% B_Jr_FR = jointToRotationJacobian(q, hip_yaw_location(3), relative_joint_vectors);
% B_Jr_BR = jointToRotationJacobian(q, hip_yaw_location(4), relative_joint_vectors);

% Visualize joint positions
robot = importrobot("magnecko.urdf");
robot.DataFormat = 'row';
subplot(6,2,[7 8 9 10 11 12]);

% % FL,FL_feet,BL,BL_feet,FR,FR_feet,BR, BR_feet
q_urdf=[q(1:3, 1)' q(4:6, 1)' q(1:3, 2)' q(4:6, 2)' q(1:3, 3)' q(4:6, 3)' q(1:3, 4)' q(4:6, 4)'];
show(robot, q_urdf, 'frames', 'on', 'PreservePlot', 0);


