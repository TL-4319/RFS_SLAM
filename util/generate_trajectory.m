function [pos, orientation, vel_body, acc_body, acc_world, rot_vel_body,rot_vel_world,time_vec] = generate_trajectory (pos_wp,...
    orientation_wp, speed_wp, dt)
    
    trajectory = waypointTrajectory(pos_wp, GroundSpeed=speed_wp, ...
        Orientation=orientation_wp, SampleRate=1/dt);
    time_vec = [];
    cur_time = -dt;
    pos = [];
    orientation = quaternion();
    vel_body = [];
    acc_body = [];
    rot_vel_body = [];
    acc_world = [];
    rot_vel_world = [];
    
    while ~isDone(trajectory)
        [cur_pos, cur_orientation, cur_vel_world, cur_acc_world, cur_rot_vel] = trajectory();
        cur_time = cur_time + dt;
        pos = horzcat(pos, cur_pos');
        orientation = vertcat(orientation, cur_orientation);
        cur_vel_body = rotateframe(cur_orientation,cur_vel_world);
        cur_acc_body = rotateframe(cur_orientation, cur_acc_world);
        cur_rot_vel_body = rotateframe(cur_orientation, cur_rot_vel);
        vel_body = horzcat(vel_body, cur_vel_body');
        acc_body = horzcat(acc_body, cur_acc_body');
        rot_vel_body = horzcat(rot_vel_body, cur_rot_vel_body');
        time_vec = horzcat(time_vec, cur_time);
        acc_world = horzcat(acc_world, cur_acc_world');
        rot_vel_world = horzcat(rot_vel_world, cur_rot_vel');
    end

end