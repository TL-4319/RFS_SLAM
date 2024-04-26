%close all
clear
clc

addpath ../util
rng(450)
%% Generate landmark map
map_size = 15;
num_landmark = 1000;
landmark_locations = vertcat((rand(1,num_landmark)) * 2 * map_size ,...
    (rand(1,num_landmark)-0.5) * 2 * map_size, (rand(1,num_landmark)-0.9) * 3);
marker_size = ones(size(landmark_locations,2),1) * 10;

%% Sensor/Robot
% Sensor properties
sensor.HFOV = deg2rad(110);
sensor.VFOV = deg2rad(10);
sensor.max_range = 15;
sensor.min_range = 0.4;
sensor.P_d = 0.7;
sensor.clutter_rate = 0;
sensor.sigma = 0.1;

%% True robot pose and measurement
% robot pose
true_pos = [10;0;0];
true_quat = quaternion([180,10,5],"eulerd","ZYX","frame");

% measurement
[meas, landmark_inFOV] = gen_noisy_3D_meas (true_pos, true_quat, landmark_locations, sensor);
meas_world = reproject_meas (true_pos, true_quat, meas);

%% Test single cluster likelihood function
% truth GM in FOV
GM_in_prev = landmark_inFOV;
marker_size = ones(size(GM_in_prev,2),1) * 10;
GM_cov = diag ([0.2, 0.2, 0.2].^2);
GM_in_prev_cov = repmat(GM_cov,1,1,size(GM_in_prev,2));
filter_meas_noise = 0.2;
R = diag([filter_meas_noise, filter_meas_noise, filter_meas_noise].^2);
GM_inten = ones(1,size(GM_in_prev,2));

% test pose
test_pos = [10;0;0];
test_quat = quaternion([180,10,180],"eulerd","ZYX","frame");
meas_world = reproject_meas (test_pos, test_quat, meas);
test_particle.pos = test_pos;
test_particle.quat = test_quat;
test_pd = 0.7;
test_clutter = 10 / (15^2 * 0.3 * pi);
%test_clutter = 3*10^-7;

[pred_meas, K, S, P, Sinv] = compute_update_terms (test_particle, GM_in_prev, GM_in_prev_cov, R);

particle_likelihood = single_cluster_likelihood(meas, pred_meas, GM_inten,Sinv, S, test_pd, test_clutter);

%% Plotting
figure(1)
draw_trajectory(test_pos, test_quat, [0;0;0], 1, 2, 2,'k',false);
set(gca, 'Zdir', 'reverse')
set(gca, 'Ydir', 'reverse')
grid on
%view([0,90])
hold on
scatter3(GM_in_prev(1,:),GM_in_prev(2,:),GM_in_prev(3,:), marker_size,'k')
scatter3(meas_world(1,:), meas_world(2,:), meas_world(3,:), 'r*')
hold off
xlabel("X");
ylabel("Y");
zlabel("Z");
axis equal