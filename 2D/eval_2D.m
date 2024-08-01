close all;
clear; 
clc;

addpath('../../util/')

%% Select simulation
% Parse simulation
[file,location] = uigetfile;
load (strcat(location,file));


%% 
time_vec = simulation.result{1,1}.time_vec;
dt = time_vec(2) - time_vec(1);

% Position 
est_pos = simulation.result{1,1}.filter_est.pos;
true_pos = simulation.result{1,1}.truth.pos;
odom_pos = simulation.result{1,1}.odom_est.pos;

est_quat = simulation.result{1,1}.filter_est.quat;
true_quat = simulation.result{1,1}.truth.quat;
odom_quat = simulation.result{1,1}.odom_est.quat;

% Convert to euler [yaw, pitch, roll]
est_euler = transpose(quat2eul(est_quat));
true_euler = transpose(quat2eul(true_quat));
odom_euler = transpose(quat2eul(odom_quat));


% Calculate distance travelled for relative metric
dist_travel = true_pos;
dist_travel(:,2:end) = true_pos(:,2:end) - true_pos(:,1:end-1);
dist_travel = vecnorm(dist_travel);
dist_travel = cumsum(dist_travel);

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
subplot (2,1,1)
plot (time_vec, abs(pos_error(1,:)),'DisplayName','PHD-SLAM')
hold on 
plot (time_vec, abs(odom_pos_error(1,:)),'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("X error")
legend

subplot (2,1,2)
plot (time_vec, abs(pos_error(2,:)),'DisplayName','PHD-SLAM')
hold on
plot (time_vec, abs(odom_pos_error(2,:)),'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Y error")

figure(2)
plot (time_vec, dist_error,'DisplayName','PHD-SLAM')
hold on
plot (time_vec, odom_dis_error,'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Distance error")
legend

figure(3)
plot (time_vec, abs(euler_error(1,:)) * 180/pi,'DisplayName','PHD-SLAM')
hold on
plot (time_vec, abs(odom_euler_error(1,:)) * 180/pi, 'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (deg)")
grid on
title("Rotational error")
legend

figure (4)
draw_trajectory(true_pos(:,end), true_quat(end,:), true_pos, 4, 2, 'k', false)
draw_trajectory(est_pos(:,end), est_quat(end,:), est_pos, 4, 2,'g',true);
draw_trajectory(odom_pos(:,end), odom_quat(end,:),odom_pos, 4, 2,'r',true);
hold on
set(gca, 'Zdir', 'reverse')
set(gca, 'Ydir', 'reverse')
grid on
view([0,90])
plot ([0,0], [0,0],'k','DisplayName','True trajectory')
plot ([0,0], [0,0],'g','DisplayName','Estimated trajectory')
plot ([0,0], [0,0],'r','DisplayName','Odometry trajectory')
legend