close all;
clear; 
clc;

addpath('util/')

%% Parse data
% Parse estimations
load ("sim-result/3d/phd-slam2-100-2.mat");

% Parse measurement table
%load ("dataset/meas_table_2.mat");


%% 
time_vec = simulation.truth.time_vec;
dt = time_vec(2) - time_vec(1);

% Position 
est_pos = simulation.est.pos;
true_pos = simulation.truth.pos;

est_quat = simulation.est.quat;
true_quat = simulation.truth.quat;

% Convert to euler [yaw, pitch, roll]
est_euler = transpose(quat2eul(est_quat));
true_euler = transpose(quat2eul(true_quat));


% Calculate distance travelled for relative metric
dist_travel = true_pos;
dist_travel(:,2:end) = true_pos(:,2:end) - true_pos(:,1:end-1);
dist_travel = vecnorm(dist_travel);
dist_travel = cumsum(dist_travel);

%% Propagate only odometry
odom_pos = simulation.truth.pos;
odom_quat = simulation.truth.quat;
odom_body_vel = simulation.odom.body_trans_vel;
odom_body_rot_vel = simulation.odom.body_rot_vel;
for tt = 2:size(time_vec,2)
    [odom_pos(:,tt),odom_quat(tt,:)] = propagate_state(odom_pos(:,tt-1), odom_quat(tt-1,:), ...
        odom_body_vel(:,tt), odom_body_rot_vel(:,tt), dt); 
end
odom_euler = transpose(quat2eul(odom_quat));
%% Error calc
pos_error = est_pos - true_pos;
dist_error = vecnorm(pos_error);

euler_error = est_euler - true_euler;

rel_trans_error = dist_error./dist_travel;
rel_trans_error(1) = 0;

rel_euler_error = euler_error;
rel_euler_error(1,:) = euler_error(1,:) ./ dist_travel;
rel_euler_error(2,:) = euler_error(2,:) ./ dist_travel;
rel_euler_error(3,:) = euler_error(3,:) ./ dist_travel;
rel_euler_error(:,1) = [0;0;0];

% Odometry only error
odom_pos_error = odom_pos - true_pos;
odom_dis_error = vecnorm(odom_pos_error);

odom_euler_error = odom_euler - true_euler;
%% Plot
figure(1)
subplot (3,1,1)
plot (time_vec, abs(pos_error(1,:)),'DisplayName','PHD-SLAM','LineWidth',2)
hold on 
plot (time_vec, abs(odom_pos_error(1,:)),'DisplayName','Odometry','LineWidth',2)
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("X error")
legend

subplot (3,1,2)
plot (time_vec, abs(pos_error(2,:)),'DisplayName','PHD-SLAM','LineWidth',2)
hold on
plot (time_vec, abs(odom_pos_error(2,:)),'DisplayName','Odometry','LineWidth',2)
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Y error")

subplot (3,1,3)
plot (time_vec, abs(pos_error(3,:)),'DisplayName','PHD-SLAM','LineWidth',2)
hold on
plot (time_vec, abs(odom_pos_error(3,:)),'DisplayName','Odometry','LineWidth',2)
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Z error")

figure(2)
plot (time_vec, dist_error,'DisplayName','PHD-SLAM','LineWidth',2)
hold on
plot (time_vec, odom_dis_error,'DisplayName','Odometry','LineWidth',2)
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Distance error")
legend

figure(3)
subplot(3,1,1)
plot (time_vec, abs(euler_error(3,:)) * 180/pi,'DisplayName','PHD-SLAM','LineWidth',2)
hold on
plot (time_vec, abs(odom_euler_error(3,:)) * 180/pi, 'DisplayName','Odometry','LineWidth',2)
xlabel("Time (s)")
ylabel("Roll error (deg)")
grid on
title("Rotational error")
legend

subplot(3,1,2)
plot (time_vec, abs(euler_error(2,:)) * 180/pi,'DisplayName','PHD-SLAM','LineWidth',2)
hold on
plot (time_vec, abs(odom_euler_error(2,:)) * 180/pi, 'DisplayName','Odometry','LineWidth',2)
xlabel("Time (s)")
ylabel("Pitch error (deg)")
grid on


subplot(3,1,3)
plot (time_vec, abs(euler_error(1,:)) * 180/pi,'DisplayName','PHD-SLAM','LineWidth',2)
hold on
plot (time_vec, abs(odom_euler_error(1,:)) * 180/pi, 'DisplayName','Odometry','LineWidth',2)
xlabel("Time (s)")
ylabel("Yaw error (deg)")
grid on


% figure (4)
% draw_trajectory(truth.pos(:,end), truth.quat(end,:), truth.pos, 4, 10, 2, 'k', false)
% draw_trajectory(est.pos(:,end), est.quat(end,:), est.pos, 4, 10, 2,'g',true);
% draw_trajectory(odom_pos(:,end), odom_quat(end,:),odom_pos, 4, 10, 2,'r',true);
% hold on
% set(gca, 'Zdir', 'reverse')
% set(gca, 'Ydir', 'reverse')
% grid on
% view([0,90])
% plot ([0,0], [0,0],'k','DisplayName','True trajectory')
% plot ([0,0], [0,0],'g','DisplayName','Estimated trajectory')
% legend

