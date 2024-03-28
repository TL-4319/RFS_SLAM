clear
clc
close all

dt = 0.2;
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

[pos, quat, trans_vel, acc_body, acc_world, rot_vel_body, rot_vel_world, imu_time_vec] = generate_trajectory2(waypoints,...
    orientation_wp, groundspeed, dt);

acc_world_with_g = acc_world;
acc_world_with_g(3,:) = acc_world_with_g(3,:) + 9.81;

%% Generate IMU data
% No bias imu
IMU_no_bias = imuSensor('accel-gyro','SampleRate',dt);
% Noise characteristic of IMU
IMU_no_bias.Accelerometer = accelparams( ...
    'MeasurementRange',19.62, ...            % m/s^2
    'NoiseDensity',0.0028);               % m/s^2 / Hz^(1/2)

IMU_no_bias.Gyroscope = gyroparams(...
    'MeasurementRange',4.363, ...   % rad/s
    'NoiseDensity', 0.00016);      % rad/s / Hz^(1/2)
[no_bias_accel, no_bias_gyro] = IMU_no_bias(acc_world', rot_vel_world', quat);

[no_bias_accel_g,~ ] =  IMU_no_bias(acc_world_with_g', rot_vel_world', quat);


%% Generate traj from odom
test_pos = pos;
test_quat = quat;

for tt = 2:size(imu_time_vec,2)
    [test_pos(:,tt),test_quat(tt)] = propagate_state(test_pos(:,tt-1), test_quat(tt-1),trans_vel(:,tt-1),rot_vel_body(:,tt-1),dt);
end

pos_dif = test_pos - pos;

plot (pos_dif')
ylim([-1 1])