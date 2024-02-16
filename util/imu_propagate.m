function [pos_pred, orientation_pred, vel_pred] = imu_propagate(pos_prev, vel_prev, orientation_prev,...
    acc_reading, gyro_reading, local_gravity,dt)
    % Calculate new orientation using gyro reading
    body_rot_rate_quat = quaternion (0, gyro_reading(1), gyro_reading(2),...
        gyro_reading(3));
    orientation_pred = orientation_prev + dt * body_rot_rate_quat * ...
        orientation_prev * 0.5;
    
    % Calculate new pos from accelerometer 
    quat_half = slerp (orientation_prev, orientation_pred, 0.5);
    delta_body_acc_quat = quaternion(0, acc_reading(1), acc_reading(2), acc_reading(3));
    world_acc_quat = quat_half * delta_body_acc_quat * conj(quat_half);
    world_acc_vec = zeros(3,1);
    [~, world_acc_vec(1), world_acc_vec(2), world_acc_vec(3)] = parts(world_acc_quat);
    % Correct by local gravity
    world_acc_vec = world_acc_vec - local_gravity';

    vel_pred = vel_prev + world_acc_vec * dt;
    pos_pred = pos_prev + vel_prev * dt + 0.5 * world_acc_vec * dt^2;
end