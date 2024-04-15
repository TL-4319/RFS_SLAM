clear
clc
close all

dt = 0.01;

addpath util/

% Generate trajectory
waypoints = [0,0,0; ... % Initial position
             20, 10, 0;...
             20, 60, 0; ...
             40, 80, 0
             70,100,0];    % Final position

orientation_wp = quaternion([0,0,0; ...
                          30,10,0;...
                          90,0,0;...
                          70,0,0;...
                          50,0,0],...
                          "eulerd","ZYX","frame");

groundspeed = ones(1,size(waypoints,1)) * 1; groundspeed(1) = 0; %Initial zero velocity

[pos, quat, trans_vel, vel_world, acc_body, acc_world, rot_vel_body, rot_vel_world, imu_time_vec] = generate_trajectory2(waypoints,...
    orientation_wp, groundspeed, dt);

imu_param.gyro_NoiseDensity = 0.0028;
imu_param.gyro_RandomWalk = 0.000086;
imu_param.accel_NoiseDensity = 0.00016;
imu_param.accel_RandomWalk = 0.000022;
imu_param.dt = dt;

%% Generate IMU data
imu_meas = generate_imu_measurements(acc_world, rot_vel_world, quat, imu_param);

%% Generate traj from odom
test_pos = pos;
test_quat = quat;


imu_pos = pos;
imu_quat = quat;
imu_vel = vel_world;
grav_vec = [0; 0; 9.81];
gyro_bias = zeros(3,size(imu_pos,2));

for tt = 2:size(imu_time_vec,2)
    %[test_pos(:,tt),test_quat(tt)] = propagate_state(test_pos(:,tt-1), test_quat(tt-1),trans_vel(:,tt-1),rot_vel_body(:,tt-1),dt);

    [imu_pos(:,tt), imu_vel(:,tt), imu_quat(tt), gyro_bias(:,tt)] = ...
        propagate_imu(imu_pos(:,tt-1), imu_vel(:,tt-1), test_quat(tt-1), ...
        imu_accel(:,tt), imu_gyro(:,tt), gyro_bias(:,tt-1), grav_vec, dt);
end

true_euler = quat2eul(quat);

imu_euler = quat2eul(imu_quat);

pos_error = abs(pos - imu_pos);
vel_error = abs(vel_world - imu_vel);
%acc_error = abs(imu_accel - acc_body_with_g);
euler_error = abs(true_euler - imu_euler) * 180/pi;

%plot (acc_error');
