function particle = propagate_particle_state_2d (particle, motion_cov, dt)
    % Sample vx, vy and theta from normal distribution 0 mean and cov 
    % This particular sampling is for 2D only
    body_vel_sample = normrnd (0, sqrt(motion_cov(1)), 3, size(particle,2));
    body_vel_sample(3,:) = zeros (1,size(particle,2));

    body_rot_vel = normrnd (0, sqrt(motion_cov(6)), 3, size(particle,2));
    body_rot_vel(1:2,:) = zeros (2,size(particle,2));

    for i = 1:size(particle,2)
        % Update quaternion
        body_rot_rate_quat = quaternion (0, body_rot_vel(1,i), body_rot_vel(2,i), body_rot_vel(3,i));
        pred_quat = particle(i).quat + dt * body_rot_rate_quat * particle(i).quat * 0.5;

        % Update pos
        quat_half = slerp (particle(i).quat, pred_quat, 0.5);
        delta_body_trans_quat = quaternion(0, body_vel_sample(1,i)*dt, body_vel_sample(2,i)*dt, body_vel_sample(3,i)*dt);
        world_trans_quat = quat_half * delta_body_trans_quat * conj(quat_half);
        world_trans_vec = zeros(3,1);
        [~, world_trans_vec(1), world_trans_vec(2), world_trans_vec(3)] = parts(world_trans_quat);
        pos = particle(i).pos + world_trans_vec;

        particle(i).pos = pos;
        particle(i).quat = pred_quat;
    end
      
end