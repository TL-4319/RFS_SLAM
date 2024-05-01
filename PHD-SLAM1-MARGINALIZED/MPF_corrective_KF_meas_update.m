function [x, P] = MPF_corrective_KF_meas_update (prev_state, pred_state, x_prev, P_prev,...
    R, acc, gyr, grav_vec, dt)
    psuedo_y = calc_psuedo_meas(prev_state, pred_state, acc, gyr, grav_vec, dt);

    F_n = calc_Fn (prev_state, dt);
    
    G_n = calc_Gn (prev_state, dt);
    
    % Predicted measurement given following measurement model
    % psuedo_y = F_n * x + G_n * v_n
    y_pred = F_n * x_prev;

    innov = psuedo_y - y_pred;
    R_mat = G_n * R * G_n';
    S = F_n * P_prev * F_n' + R_mat; 
    S = (S + S')/2;
    K = P_prev * F_n' * pinv(S);
    x = x_prev + K * innov;
    temp = (eye(6) - K * F_n);
    P = temp * P_prev * temp' + K * R_mat * K';
end

function psuedo_y = calc_psuedo_meas (prev_state, pred_state, acc, gyr, grav_vec, dt)
    % Calculate pseudomeasurement for the MPF linear subpart
    % y_k = x_k+1 - f_k
    f_k = calc_non_linear_f (prev_state, acc, gyr, grav_vec, dt);
    
    x_non_linear = [pred_state.pos; pred_state.vel; compact(pred_state.quat)'];

    psuedo_y = x_non_linear - f_k;
end

function f_k = calc_non_linear_f (prev_state, acc, gyr, grav_vec, dt)
    % Calculate the expected non-linear state without contribution from
    % linear sub-part
    % Implement 5.2b on page 109 in Blesser thesis
    f_k = zeros(10,1);

    pos = prev_state.pos;
    quat = compact(prev_state.quat);
    vel = prev_state.vel;
    
    RotM = quat2rot(quat, "point"); % Rotation matrix from body to world
    Sq = quat2Sq(quat);

    f_k(1:3) = pos + dt * vel + 0.5 * dt^2 * (RotM * acc - grav_vec);
    f_k(4:6) = vel + dt * (RotM * acc - grav_vec);
    f_k(7:10) = quat' - 0.5 * dt * Sq * gyr;
end

function F_n = calc_Fn (prev_state, dt)
    % Jacobian of non-linear dynamic wrt linear sub part
    % Implement 5.2b on page 109 in Blesser thesis
    % Equivalent to measurement Jacobian H wrt to bias
    
    %% Only gyro bias
    % quat = compact(prev_state.quat);
    % Sq = quat2Sq(quat);
    % 
    % F_n = zeros(10,3);
    % 
    % F_n (7:10, 1:3) = 0.5 * dt * Sq;
    
    %% Gyro and acc bias
    quat = compact(prev_state.quat);
    Sq = quat2Sq(quat);
    Rotm = quat2rot(quat,"point");

    F_n = zeros(10,6);
    
    F_n(1:3,4:6) = -0.5 * dt^2 * Rotm;

    F_n(4:6,4:6) = -dt * Rotm;

    F_n (7:10, 1:3) = 0.5 * dt * Sq;
end

function G_n = calc_Gn (prev_state, dt)
    % Jacobian of non-linear dynamic wrt non-linear noise
    % Implement 5.2b on page 109 in Blesser thesis
    % Equivalent to measurement Jacobian H wrt to v used to calculate R
    quat = compact(prev_state.quat);
    Sq = quat2Sq(quat);
    RotM = quat2rot(quat, "point");

    G_n = zeros(10,6);

    G_n(1:3,4:6) = -0.5 * dt^2 * RotM;

    G_n(4:6,4:6) = -dt * RotM;

    G_n(7:10,1:3) = 0.5 * dt * Sq;
end

