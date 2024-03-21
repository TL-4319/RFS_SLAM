function [pos, quat] = propagate_state (prev_pos, prev_quat, body_trans_vel, body_rot_vel, dt)
    body_rot_rate_quat = quaternion (0, body_rot_vel(1), body_rot_vel(2), body_rot_vel(3));
    quat = prev_quat + dt * body_rot_rate_quat * prev_quat * 0.5;
    quat_half = slerp (prev_quat, quat, 0.5);
    delta_body_trans_quat = quaternion(0, body_trans_vel(1)*dt, body_trans_vel(2)*dt, body_trans_vel(3)*dt);
    world_trans_quat = quat_half * delta_body_trans_quat * conj(quat_half);
    world_trans_vec = zeros(3,1);
    [~, world_trans_vec(1), world_trans_vec(2), world_trans_vec(3)] = parts(world_trans_quat);
    pos = prev_pos + world_trans_vec;
end