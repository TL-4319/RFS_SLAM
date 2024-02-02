close all;
clear; 
clc;


%% Parse data
% Parse estimations
load ("PHD_SLAM1/sim_data_output/est_output_1000_part_dataset2.mat");

% Parse measurement table
load ("dataset/meas_table_2.mat");

% Parse truth data
load ("dataset/truth_2.mat")

%% 
time_vec = truth.time_vec;

% Position 
est_pos = est_traj.hist;
true_pos = truth.pos;

est_quat = est_traj.quat_hist;
true_quat = truth.quat;

% Convert to euler [yaw, pitch, roll]
est_euler = transpose(quat2eul(est_quat));
true_euler = transpose(quat2eul(true_quat));

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
%% Plot
figure(1)
subplot (2,1,1)
plot (time_vec, abs(pos_error(1,:)))
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("X error")

subplot (2,1,2)
plot (time_vec, abs(pos_error(2,:)))
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Y error")

figure(2)
plot (time_vec, dist_error)
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Distance error")

figure(3)
plot (time_vec, abs(euler_error(1,:)) * 180/pi)
xlabel("Time (s)")
ylabel("error (deg)")
grid on
title("Rotational error")


