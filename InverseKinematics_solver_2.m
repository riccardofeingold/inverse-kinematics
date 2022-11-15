function [q_invKin] = InverseKinematics_solver(I_r_IE_des, C_IE_des, q_0, tol, robot, hip_yaw_location, leg_dimensions, body_orientation, distance_hip_joints, stationary_feet, relative_joint_vectors)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% 0. Setup
it = 0;
max_it = 100;       % Set the maximum number of iterations. 
lambda = 0.001;     % Damping factor
alpha = 0.5;        % Update rate

% 1. start configuration
q = q_0;

I_r_IE_des_BR = findBaseToFootVector(q(1:3,4), hip_yaw_location(1:3, 4), leg_dimensions, distance_hip_joints);
I_r_IE_des_FR = findBaseToFootVector(q(1:3,3), hip_yaw_location(1:3, 3), leg_dimensions, distance_hip_joints);
I_r_IE_des_BL = findBaseToFootVector(q(1:3,2), hip_yaw_location(1:3, 2), leg_dimensions, distance_hip_joints);
% 2. Iterate until terminating condition.
while (it==0 || (norm(dr)>tol && it < max_it))
    % 3. evaluate Jacobian for current q
    [T_IB, T_BI] = getTransformIB(q, body_orientation, leg_dimensions, distance_hip_joints, hip_yaw_location, stationary_feet);
    C_IB = T_IB(1:3, 1:3);
    
    % FL
    % Calculate all the jacobians needed
    B_r_BFL = findBaseToFootVector(q(1:3, 1), hip_yaw_location(1:3, 1), leg_dimensions, distance_hip_joints);
    B_Jp_FL = jointToPositionJacobian(q(1:3, 1), hip_yaw_location(1:3,1), relative_joint_vectors);
    B_Jp_FL = [
        B_Jp_FL zeros(3,3) zeros(3,3) zeros(3,3);
    ];
%     B_Jr_FL = jointToRotationJacobian(q(1:3, 1), hip_yaw_location(1:3, 1), relative_joint_vectors);
%     B_Jr_FL = [
%         B_Jr_FL zeros(3,3) zeros(3,3) zeros(3,3)
%     ];
    % Position error
    I_r_IE_current = findBaseToFootVector(q(1:3,1), hip_yaw_location(1:3, 1), leg_dimensions, distance_hip_joints)
    dr = I_r_IE_des - I_r_IE_current; 
    
    % BL
    % Calculate all the jacobians needed
    B_r_BBL = findBaseToFootVector(q(1:3, 2), hip_yaw_location(1:3, 2), leg_dimensions, distance_hip_joints);
    B_Jp_BL = jointToPositionJacobian(q(1:3, 2), hip_yaw_location(1:3,2), relative_joint_vectors);
    B_Jp_BL = [
        zeros(3,3) B_Jp_BL zeros(3,3) zeros(3,3);
    ];
%     B_Jr_BL = jointToRotationJacobian(q(1:3, 2), hip_yaw_location(1:3, 2), relative_joint_vectors);
%     B_Jr_BL = [
%         zeros(3,3) B_Jr_BL zeros(3,3) zeros(3,3);
%     ];
    I_r_IE_current_BL = findBaseToFootVector(q(1:3,2), hip_yaw_location(1:3, 2), leg_dimensions, distance_hip_joints)
    dr_BL = I_r_IE_des_BL - I_r_IE_current_BL; 

    % FR
    % Calculate all the jacobians needed
    B_r_BFR = findBaseToFootVector(q(1:3, 3), hip_yaw_location(1:3, 3), leg_dimensions, distance_hip_joints);
    B_Jp_FR = jointToPositionJacobian(q(1:3, 3), hip_yaw_location(1:3,3), relative_joint_vectors);
    B_Jp_FR = [
        zeros(3,3) zeros(3,3) B_Jp_FR zeros(3,3);
    ];
%     B_Jr_FR = jointToRotationJacobian(q(1:3, 3), hip_yaw_location(1:3, 3), relative_joint_vectors);
%     B_Jr_FR = [
%         zeros(3,3) zeros(3,3) B_Jr_FR zeros(3,3)
%     ];
    I_r_IE_current_FR = findBaseToFootVector(q(1:3,3), hip_yaw_location(1:3, 3), leg_dimensions, distance_hip_joints)
    dr_FR = I_r_IE_des_FR - I_r_IE_current_FR; 

    % BR
    % Calculate all the jacobians needed
    B_r_BBR = findBaseToFootVector(q(1:3, 4), hip_yaw_location(1:3, 4), leg_dimensions, distance_hip_joints);
    B_Jp_BR = jointToPositionJacobian(q(1:3, 4), hip_yaw_location(1:3,4), relative_joint_vectors);
    B_Jp_BR = [
        zeros(3,3) zeros(3,3) zeros(3,3) B_Jp_BR;
    ];
%     B_Jr = jointToRotationJacobian(q(1:3, 4), hip_yaw_location(1:3, 4), relative_joint_vectors);
%     B_Jr = [
%         zeros(3,3) zeros(3,3) zeros(3,3) B_Jr;
%     ];
    I_r_IE_current_BR = findBaseToFootVector(q(1:3,4), hip_yaw_location(1:3, 4), leg_dimensions, distance_hip_joints)
    dr_BR = I_r_IE_des_BR - I_r_IE_current_BR; 

    % reshaping joint matrix to vector
    q_vector = [
        zeros(6,1);
        reshape(q,[],1);
    ];

    % finding I_Jp
    I_Jp_FL = [eye(3) -C_IB*skewMatrix(B_r_BFL) C_IB*B_Jp_FL];
    I_Jp_BL = [eye(3) -C_IB*skewMatrix(B_r_BBL) C_IB*B_Jp_BL];
    I_Jp_FR = [eye(3) -C_IB*skewMatrix(B_r_BFR) C_IB*B_Jp_FR];
    I_Jp_BR = [eye(3) -C_IB*skewMatrix(B_r_BBR) C_IB*B_Jp_BR];
    I_Jp_B = [zeros(3, 15) eye(3,3)];

    I_Jp = [
        I_Jp_FL;
        I_Jp_BL;
        I_Jp_FR;
        I_Jp_BR;
    ];
    
    % Kinematic Controller
    kp = 5;
    v_des = zeros(12,1);

    dr = [
        dr;
        dr_BL;
        dr_FR;
        dr_BR;
    ];

    N = eye(18,18) - pinv(I_Jp)*I_Jp;
    v_command_1 = v_des + kp*dr;
    v_command_2 = zeros(3,1);
    Dq = pinv(I_Jp)*v_command_1 + N*pinv(I_Jp_B*N)*(v_command_2 - I_Jp_B*pinv(I_Jp)*v_command_1);

    q_vector = q_vector + Dq*0.1;

    % Turn q_vector into matrix
    q = reshape(q_vector(7:18), 3, 4);
    % Update robot
    q_urdf=[q(1:3, 1)' 0 0 0 q(1:3, 2)' 0 0 0 q(1:3, 3)' 0 0 0 q(1:3, 4)' 0 0 0];
    show(robot, q_urdf, 'frames', 'on', 'PreservePlot', 0);

    drawnow;
    pause(0.1);
    
    it = it+1;
    q_invKin = q_vector;
end
end

