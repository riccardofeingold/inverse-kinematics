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

% Joint variables
hip_yaw = pi/4;
hip_pitch = 0;
knee_pitch = 0;

q = [hip_yaw, hip_pitch, knee_pitch];

% Body Orientation (measured e.g. by an IMU)
body_roll = pi/10; % around x-axis
body_pitch = pi/5; % around y-axis
body_yaw = pi/20; % around z-axis

body_orientation = [body_roll, body_pitch, body_yaw];

C_IB = eul2rotm(body_orientation);

foot_positions = zeros(3,3);
for k=1:3
    foot_positions(1:3, k) = findBaseToFootVector(q, body_dimensions, leg_dimensions, distance_hip_joints);
end


% relative vector from B to I frame
r_BI = 1/3 * (foot_positions(1:3, 1) + foot_positions(1:3, 2) + foot_positions(1:3, 3));

% calculate homogeneous transformation from I to B frame => find r_IB
T_BI = [
    C_IB' r_BI;
    zeros(1,3) 1;
];

T_IB = inv(T_BI);
r_IB = T_IB(1:3, 3);


