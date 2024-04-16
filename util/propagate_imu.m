function [pos, vel, quat, gyro_bias, accel_bias] = propagate_imu (prev_pos, prev_vel, prev_quat, imu_accel, imu_gyro, prev_gyro_bias, prev_accel_bias, grav_vec, dt)
    % Propagate quaternion - https://ahrs.readthedocs.io/en/latest/filters/angular.html
    body_rot_vel = imu_gyro - prev_gyro_bias;
    quat = rotate_quat(prev_quat, body_rot_vel, dt);
    
    prev_quat_vec = compact(prev_quat);
    RotM = quat2rot(prev_quat_vec, "point");
    
    % imu measurement to body accel (due to imu report specific force)
    imu_accel = -imu_accel + prev_accel_bias;

    if norm(grav_vec) > 0.1
        pos = prev_pos + dt * prev_vel + dt^2 *0.5 * RotM * imu_accel + dt^2 * 0.5 * grav_vec; 
        vel = prev_vel + dt * RotM * imu_accel + dt * grav_vec;
    else
        % no gravity
        pos = prev_pos + dt * prev_vel + dt^2 *0.5 * RotM * imu_accel; 
        vel = prev_vel + dt * RotM * imu_accel;
    end
    gyro_bias = prev_gyro_bias;
    accel_bias = prev_accel_bias;
end

function quat = rotate_quat(prev_quat, body_rot_vel, dt)
    prev_quat_vec = compact(prev_quat)'; % Convert to vector type for ease of implementation
    
    OMEGA = calc_omega(body_rot_vel);
    quat_vec = (eye(4) + 0.5 * OMEGA * dt) * prev_quat_vec;
    quat = quaternion(quat_vec');
end

function OMEGA = calc_omega(body_rot_vel)
    skew_mat = calc_skew_mat(body_rot_vel);
    OMEGA = zeros(4);
    OMEGA (2:4,1) = body_rot_vel;
    OMEGA (1, 2:4) = -body_rot_vel';
    OMEGA (2:4, 2:4) = -skew_mat;
end

function skew_mat = calc_skew_mat (body_rot_vel)
    wz = body_rot_vel(3);
    wy = body_rot_vel(2);
    wx = body_rot_vel(1);

    skew_mat = [0, -wz, wy;
               wz,   0,-wx;
              -wy,  wx,  0];
end