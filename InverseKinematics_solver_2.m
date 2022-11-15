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

% 2. Iterate until terminating condition.
while (it==0 || (norm(dxe)>tol && it < max_it))
    % 3. evaluate Jacobian for current q
    [T_IB, T_BI] = getTransformIB(q, body_orientation, leg_dimensions, distance_hip_joints, hip_yaw_location, stationary_feet);
    C_IB = T_IB(1:3, 1:3);
    
    if stationary_feet(1) == 1 % FL
        % Calculate all the jacobians needed
        B_r_BQ = findBaseToFootVector(q(1:3, 1), hip_yaw_location(1:3, 1), leg_dimensions, distance_hip_joints);
        B_Jp = jointToPositionJacobian(q(1:3, 1), hip_yaw_location(1:3,1), relative_joint_vectors);
        B_Jp = [
            B_Jp zeros(3,3) zeros(3,3) zeros(3,3);
        ];
        B_Jr = jointToRotationJacobian(q(1:3, 1), hip_yaw_location(1:3, 1), relative_joint_vectors);
        B_Jr = [
            B_Jr zeros(3,3) zeros(3,3) zeros(3,3)
        ];

        % Position error
        I_r_IE_current = findBaseToFootVector(q(1:3,1), hip_yaw_location(1:3, 1), leg_dimensions, distance_hip_joints)
        dr = I_r_IE_des - I_r_IE_current; 
        [foot_position, C_BFL] = findBaseToFootVector(q(1:3, 1), hip_yaw_location(1:3,1), leg_dimensions, distance_hip_joints);
        [T_IB, T_BI] = getTransformIB(q, body_orientation, leg_dimensions, distance_hip_joints, hip_yaw_location, stationary_feet);
        C_IFL = T_IB(1:3, 1:3)*C_BFL;
        rotation_error = C_IE_des*C_IFL';
        dph = rotMatToRotVec(rotation_error); 
        % pose error
        dxe = [dr; dph];
    elseif stationary_feet(2) == 1 % BL
        % Calculate all the jacobians needed
        B_r_BQ = findBaseToFootVector(q(1:3, 2), hip_yaw_location(1:3, 2), leg_dimensions, distance_hip_joints);
        B_Jp = jointToPositionJacobian(q(1:3, 2), hip_yaw_location(1:3,2), relative_joint_vectors);
        B_Jp = [
            zeros(3,3) B_Jp zeros(3,3) zeros(3,3);
        ];
        B_Jr = jointToRotationJacobian(q(1:3, 2), hip_yaw_location(1:3, 2), relative_joint_vectors);
        B_Jr = [
            zeros(3,3) B_Jr zeros(3,3) zeros(3,3)
        ];

        % Position error
        I_r_IE_current = findBaseToFootVector(q(1:3,2), hip_yaw_location(1:3, 2), leg_dimensions, distance_hip_joints)
        dr = I_r_IE_des - I_r_IE_current; 
        [foot_position, C_BFL] = findBaseToFootVector(q(1:3, 2), hip_yaw_location(1:3,2), leg_dimensions, distance_hip_joints);
        [T_IB, T_BI] = getTransformIB(q, body_orientation, leg_dimensions, distance_hip_joints, hip_yaw_location, stationary_feet);
        C_IFL = T_IB(1:3, 1:3)*C_BFL;
        rotation_error = C_IE_des*C_IFL';
        dph = rotMatToRotVec(rotation_error); 
        % pose error
        dxe = [dr; dph];
    elseif stationary_feet(3) == 1 % FR
        % Calculate all the jacobians needed
        B_r_BQ = findBaseToFootVector(q(1:3, 3), hip_yaw_location(1:3, 3), leg_dimensions, distance_hip_joints);
        B_Jp = jointToPositionJacobian(q(1:3, 3), hip_yaw_location(1:3,3), relative_joint_vectors);
        B_Jp = [
            zeros(3,3) zeros(3,3) B_Jp zeros(3,3);
        ];
        B_Jr = jointToRotationJacobian(q(1:3, 3), hip_yaw_location(1:3, 3), relative_joint_vectors);
        B_Jr = [
            zeros(3,3) zeros(3,3) B_Jr zeros(3,3)
        ];

        % Position error
        I_r_IE_current = findBaseToFootVector(q(1:3,3), hip_yaw_location(1:3, 3), leg_dimensions, distance_hip_joints)
        dr = I_r_IE_des - I_r_IE_current; 
        [foot_position, C_BFL] = findBaseToFootVector(q(1:3, 3), hip_yaw_location(1:3,3), leg_dimensions, distance_hip_joints);
        [T_IB, T_BI] = getTransformIB(q, body_orientation, leg_dimensions, distance_hip_joints, hip_yaw_location, stationary_feet);
        C_IFL = T_IB(1:3, 1:3)*C_BFL;
        rotation_error = C_IE_des*C_IFL';
        dph = rotMatToRotVec(rotation_error); 
        % pose error
        dxe = [dr; dph];
    elseif stationary_feet(4) == 1 % BR
        % Calculate all the jacobians needed
        B_r_BQ = findBaseToFootVector(q(1:3, 4), hip_yaw_location(1:3, 4), leg_dimensions, distance_hip_joints);
        B_Jp = jointToPositionJacobian(q(1:3, 4), hip_yaw_location(1:3,4), relative_joint_vectors);
        B_Jp = [
            zeros(3,3) zeros(3,3) zeros(3,3) B_Jp;
        ];
        B_Jr = jointToRotationJacobian(q(1:3, 4), hip_yaw_location(1:3, 4), relative_joint_vectors);
        B_Jr = [
            zeros(3,3) zeros(3,3) zeros(3,3) B_Jr;
        ];

        % Position error
        I_r_IE_current = findBaseToFootVector(q(1:3,4), hip_yaw_location(1:3, 4), leg_dimensions, distance_hip_joints)
        dr = I_r_IE_des - I_r_IE_current; 
        [foot_position, C_BFL] = findBaseToFootVector(q(1:3, 4), hip_yaw_location(1:3,4), leg_dimensions, distance_hip_joints);
        [T_IB, T_BI] = getTransformIB(q, body_orientation, leg_dimensions, distance_hip_joints, hip_yaw_location, stationary_feet);
        C_IFL = T_IB(1:3, 1:3)*C_BFL;
        rotation_error = C_IE_des*C_IFL';
        dph = rotMatToRotVec(rotation_error); 
        % pose error
        dxe = [dr; dph];
    end
    
    % reshaping joint matrix to vector
    q_vector = [
        zeros(6,1);
        reshape(q,[],1);
    ];

    % finding I_Jp
    I_Jp = [eye(3) -C_IB*skewMatrix(B_r_BQ) C_IB*B_Jp];
    I_Jr = [zeros(3,3) C_IB C_IB*B_Jr];

    I_J = [I_Jp; I_Jr];
    
    % 4. Update the psuedo inverse
    I_J_pinv = size(pinv(I_J, lambda))
    I_Jr_pinv = pinv(I_Jr, lambda);
    I_Jp_pinv = pinv(I_Jp, lambda);
    
    % joint constraints
    J_cFL = [
        zeros(6,6) ones(6,3) zeros(6, 9);
    ];
    J_cFR = [
        zeros(6,6) zeros(6,3) ones(6,3) zeros(6, 6);
    ];

    J_c = [
        J_cFL;
        J_cFR;
    ];
    
    J_c = [
        zeros(1,6) zeros(1,3) ones(1,3) zeros(1, 6);
        zeros(1,6) zeros(1,3) zeros(1,3) ones(1,3) zeros(1,3);
        zeros(1,6) zeros(1,3) zeros(1,3) zeros(1,3) ones(1,3);
        zeros(3,6) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3);
    ];

    N_cFL = eye(18,18) - pinv(J_cFL)*J_cFL;
    N_cFR = eye(18,18) - pinv(J_cFR)*J_cFR;
    N_c = eye(18,18) - pinv(J_c)*J_c;
    I_Jp_pinv = pinv(I_Jp*N_c, lambda);

    % N_redundancy = eye(18,18) - I_J_pinv*I_J;
    % 6. Update the generalized coordinates
    % inverse kinematics using whole gemetric jacobian
    % % q_vector = q_vector + alpha*I_J_pinv*dxe;
    
    % shortest path method using rotation error vector
    % % kpr = 0.5;
    % % q_vector = q_vector + kpr*I_Jr_pinv*dph;
    
    % trajectory controller using dq
%     kp = 5;
% 
%     v_des = zeros(3,1);
%     v_command = v_des + kp*dr;
%     Dq = I_Jp_pinv*v_command;
%     
%     q_vector = q_vector + Dq*0.1;
    
    % weighted matrix
%     kp=5;
%     W = diag([1,1,1,1,1,1,2,2,2,2,2,2]);
%     I_J = [
%         I_J;
%         J_c;
%     ];
%     size(I_J')
%     I_J_pinv = size(pinv(I_J'*W*I_J)*I_J'*W);
%     v_des = zeros(3,1);
%     v_command = v_des + kp*dr;
%     Dq = I_J_pinv*v_command;
% 
%     q_vector = q_vector + Dq*0.1;
    % Multitask with priorization
%     w1 = [1 1 1 1 1 1]';
%     w2 = [0 0 0 0 0 0]';
%     Dq = I_J_pinv*w1 + N_c*pinv(J_c*N_c, lambda)*(w2 - J_c*I_J_pinv*w1);
%     
%     q_vector = q_vector + Dq*0.1;

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

