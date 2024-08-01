%% Generate simulated 2D datasets to test RFS-SLAM algorhitm
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

% Enable or disable visualization
draw = false;

%% Generate landmark map - MAP ARE RANDOM
map_size = 50;
num_landmark = 500;
landmark_locations = (rand(num_landmark, 3) - 0.2) * 2 * map_size;
landmark_locations(:,3) = zeros(1,size(landmark_locations,1));
landmark_locations = landmark_locations';

%% Define principal sampling rate of simulation
% This is equivalent to the sampling rate of the fastest available sensor
% you want to simulate (base 30Hz for camera or 100Hz if IMU included)
data_rate_hz = 30;

%% Define trajectory
% Generate trajectory - EDIT HERE TO CHANGE ROBOT PATH
waypoints = [0,0,0; ... % Initial position
             10, 0, 0; ...
             11, 0, 0; ...
             12, 0, 0; ...
             50,40,0];    % Final position

orientation_wp = quaternion([1,0,0; ...
                          1,0,0;...
                          1,0,0;...
                          4,0,0;...
                          45,0,0],...
                          "eulerd","ZYX","frame");

% Define ground speed. This can be constant or variable
ground_speed_mps = 1;
groundspeed = ones(1,size(waypoints,1)) * 1; groundspeed(1) = 0; %Initial zero velocity

% Generate pose
dt = 1/data_rate_hz;
[pos, quat, trans_vel_body, acc_body, acc_world, rot_vel_body, ...
    rot_vel_world, time_vec] = generate_trajectory(waypoints,...
    orientation_wp, groundspeed, dt);

%%
dataset.pos = pos;
dataset.quat = quat;
dataset.trans_vel_body = trans_vel_body;
dataset.rot_vel_body = rot_vel_body;
dataset.accel_body = acc_body;

% Extra measurement can be used to simulate IMU
dataset.rot_vel_world = rot_vel_world;
dataset.accel_world = acc_world;

dataset.time_vec = time_vec;
dataset.landmark_locations = landmark_locations;

%% Visualize for verification 
if draw
    for kk = 1:size(time_vec,2)
        figure(1)
        draw_trajectory(pos(:,kk), quat(kk,:),pos(:,1:kk-1), 5, 2, 'k',false)
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