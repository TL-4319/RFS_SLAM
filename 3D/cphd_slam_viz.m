close all;
clear; 
clc;

addpath('../util/')

load('sim_result/sim-20250727-1024-c20.mat');

%%
%kk = 431;
%kk = 531;
%kk = 561;
kk = 824;
i = 20;
landmark_locations = simulation.truth.cummulative_landmark_in_FOV{83,1};
pos = simulation.truth.pos;
quat = simulation.truth.quat;
[~,sensor_ind] = min(abs(simulation.truth.sensor_time_vec - simulation.truth.time_vec(kk)));
map_est_struct = simulation.result{i}.filter_est.map_est{sensor_ind,1};
map_est = map_est_struct.feature_pos;
est_pos = simulation.result{i}.filter_est.pos;
est_quat = simulation.result{i}.filter_est.quat;
odom_pos = simulation.result{i}.odom_est.pos;
odom_quat = simulation.result{i}.odom_est.quat;

figure(1)
draw_trajectory(pos(:,kk), quat(kk,:),pos(:,1:kk-1), 1, 2, 'k',false)
draw_trajectory(est_pos(:,kk), est_quat(kk,:), est_pos(:,1:kk), 1, 2, 'g',true);
draw_trajectory(odom_pos(:,kk), odom_quat(kk,:), odom_pos(:,1:kk), 1, 2, 'r',true);
set(gca, 'Zdir', 'reverse')
set(gca, 'Ydir', 'reverse')
grid on
view([0,90])
hold on
scatter3(landmark_locations(1,:),landmark_locations(2,:),landmark_locations(3,:),'k')
plot_3D_phd(map_est_struct, 10, 0.2, 1, 2)
scatter3(map_est(1,:), map_est(2,:), map_est(3,:),...
            ones(size(map_est,2),1) * 20,'r+')
xlabel("X");
ylabel("Y");
zlabel("Z");

axis equal
modify_figure(20)