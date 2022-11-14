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
    elseif stationary_feet(2) == 1 % BL
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
    I_J_pinv = pinv(I_J, lambda);
    
    % 6. Update the generalized coordinates
    q_vector = q_vector + alpha*I_J_pinv*dxe;
    
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

