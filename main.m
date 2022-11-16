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
    %q_feet_FL q_feet_BL q_feet_FR q_feet_BR;
];

% Which feet are not going to move
% 1 = moves; 0 = not moving
% stationary_feet = [FL, BL, FR, BR]
stationary_feet = [1, 0, 0, 0];

% Body Orientation (measured e.g. by an IMU)
body_roll = pi/10; % around x-axis
body_pitch = pi/5; % around y-axis
body_yaw = pi/20; % around z-axis

body_orientation = [body_roll, body_pitch, body_yaw];

% relative vectors from one joint frame to the next 
r_hip_yaw_hip_pitch = [distance_hip_joints; 0; 0];
r_hip_pitch_knee_pitch = [upper_leg; 0; 0];
r_knee_pitch_foot = [lower_leg; 0; 0];

relative_joint_vectors = [r_hip_yaw_hip_pitch r_hip_pitch_knee_pitch r_knee_pitch_foot];

% Visualize joint positions
robot = importrobot("magnecko.urdf");
robot.DataFormat = 'row';

% % FL,FL_feet,BL,BL_feet,FR,FR_feet,BR, BR_feet
q_urdf=[q(1:3, 1)' 0 0 0 q(1:3, 2)' 0 0 0 q(1:3, 3)' 0 0 0 q(1:3, 4)' 0 0 0];
show(robot, q_urdf, 'frames', 'on', 'PreservePlot', 0);

% inverse kinematics
I_r_des = [2, 2, -1]';
I_C_des = zeros(3,3);
q_0 = q;
tol = 0.1;


version = 2;
if version == 1
    q_invKin = InverseKinematics_solver(I_r_des, I_C_des, q_0, tol, robot, hip_yaw_location, leg_dimensions, body_orientation, distance_hip_joints, [1 0 0 0], relative_joint_vectors)
    q_0 = [
        zeros(6,1);
        reshape(q,[],1);
    ];
    q_error = q_0 - q_invKin
    q_invKin = reshape(q_invKin(7:18), 3,4);
    q_invKin = InverseKinematics_solver(I_r_des, I_C_des, q_invKin, tol, robot, hip_yaw_location, leg_dimensions, body_orientation, distance_hip_joints, [0 1 0 0], relative_joint_vectors)
    q_invKin = reshape(q_invKin(7:18), 3,4);
    q_invKin = InverseKinematics_solver(I_r_des, I_C_des, q_invKin, tol, robot, hip_yaw_location, leg_dimensions, body_orientation, distance_hip_joints, [0 0 1 0], relative_joint_vectors)
    q_invKin = reshape(q_invKin(7:18), 3,4);
    q_invKin = InverseKinematics_solver(I_r_des, I_C_des, q_invKin, tol, robot, hip_yaw_location, leg_dimensions, body_orientation, distance_hip_joints, [0 0 0 1], relative_joint_vectors)
elseif version == 2
    q_invKin = InverseKinematics_solver_2(I_r_des, I_C_des, q_0, tol, robot, hip_yaw_location, leg_dimensions, body_orientation, distance_hip_joints, [1 0 0 0], relative_joint_vectors)
end
