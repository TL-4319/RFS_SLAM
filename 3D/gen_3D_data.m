%% Generate simulated 3D datasets to test RFS-SLAM algorhitm
% Data in truth includes pose, odometry, and map
% Manual save the dataset struct for usage with RFS-SLAM algorithms in
% /generated_datasets
% by Tuan Luong - tdluong@crimson.ua.edu 07/23/2024
close all;
clear;
clc

% Add utilities functions
addpath("../util/")

% Seed the RNG
rng(100);

% Enable or disable visualization. False is faster
draw = false;

%% Generate landmark map - MAP ARE RANDOM
map_size = 20;
num_landmark = 500;
min_dist_betwee_landmark = 0.3;
landmark_locations = (rand(num_landmark, 3) - 0.2) * 2 * map_size;
% Remove points that has xy coordinates too close together
ii = 1;
while ii < num_landmark
    cur_xy = landmark_locations(ii,1:2);
    
    dist_to_other = landmark_locations(:,1:2);
    dist_to_other(ii,:) = [];
    dist_to_other = dist_to_other - cur_xy;
    dist_to_other = (dist_to_other(:,1).^2 + dist_to_other(:,2).^2).^0.5;
    ind_too_close = find(dist_to_other < min_dist_betwee_landmark);
    landmark_locations(ind_too_close,:) = [];
    num_landmark = num_landmark - size(ind_too_close,1);
    ii = ii + 1;
end

landmark_locations(:,3) = (rand(num_landmark,1) - 0.5) * 0.3 ; % This aim to simulate planetary env where features are terrain based on ground
landmark_locations = landmark_locations';

%% Define principal sampling rate of simulation
% This is equivalent to the sampling rate of the fastest available sensor
% you want to simulate

data_rate_hz = 8; % 8hz value from NASA/JPL Surface attitude position and pointing system

%% Define trajectory
% Generate trajectory - EDIT HERE TO CHANGE ROBOT PATH
waypoints = [0,0,0; ... % Initial position
             1, 0, 0; ...
             3, 0, 0; ...
             4, 0, 0; ...
             15,10,0];    % Final position

orientation_wp = quaternion([0,0,0; ...
                          0,0,0;...
                          0,0,0;...
                          1,0,0;...
                          70,0,0],...
                          "eulerd","ZYX","frame");

% Define ground speed. This can be constant or variable
ground_speed_mps = 0.2;
groundspeed = ones(1,size(waypoints,1)) * ground_speed_mps; groundspeed(1) = 0; %Initial zero velocity

% Generate pose
dt = 1/data_rate_hz;
[pos, quat, trans_vel_body, acc_body, acc_world, rot_vel_body, ...
    rot_vel_world, time_vec] = generate_trajectory(waypoints,...
    orientation_wp, groundspeed, dt);

%%
% Here pos is the robot body pose. Sensor pose is annotated
dataset.pos = pos;
dataset.quat = quat;
dataset.trans_vel_body = trans_vel_body;
dataset.rot_vel_body = rot_vel_body;
dataset.accel_body = acc_body;

% Set sensor pose in robot frame
dataset.pos_body_sensor = [1;0;-1.5];
dataset.quat_body_sensor = quaternion([0, -25, 0],"eulerd","ZYX","frame");

% Calculate sensor pose 
dataset.pos_sensor = dataset.pos;
dataset.quat_sensor = dataset.quat;
for ii = 1:size(dataset.pos,2)
    dataset.pos_sensor(:,ii) = dataset.pos(:,ii) +...
        transpose(rotatepoint(dataset.quat(ii),dataset.pos_body_sensor'));
    dataset.quat_sensor(ii) = quatmultiply(dataset.quat_body_sensor, dataset.quat(ii));
end

% Extra measurement can be used to simulate IMU
dataset.rot_vel_world = rot_vel_world;
dataset.accel_world = acc_world;

dataset.time_vec = time_vec;
dataset.landmark_locations = landmark_locations;

%% Visualize for verification 
if draw
    for kk = 1:size(time_vec,2)
        figure(1)
        draw_trajectory(pos(:,kk), quat(kk,:),pos(:,1:kk-1), 2, 2, 'k',false)
        draw_trajectory(dataset.pos_sensor(:,kk), dataset.quat_sensor(kk), pos(:,1:kk-1), 2, 2,'none',true)
        set(gca, 'Zdir', 'reverse')
        set(gca, 'Ydir', 'reverse')
        grid on
        view([0,90])
        hold on
        scatter3(landmark_locations(1,:),landmark_locations(2,:),landmark_locations(3,:),'k')
        xlabel("X");
        ylabel("Y");
        zlabel("Z");
        axis equal
        title_str = sprintf("i = %d", kk);
        title (title_str)
        drawnow
    end
end