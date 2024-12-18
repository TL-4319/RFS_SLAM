close all;
%clear; 
clc;

addpath('../util/')

%% Select simulation
% Parse simulation
[file,location] = uigetfile;
load (strcat(location,file));


%% 
time_vec = simulation.truth.time_vec;
dt = time_vec(2) - time_vec(1);

% Pre allocate traj metrics
slam_pos_error = zeros(3,size(simulation.truth.pos,2));
slam_eul_error = slam_pos_error;
odom_pos_error = slam_pos_error;
odom_euler_error = slam_pos_error;

% Preallocate mapping error metrics
true_card = zeros(1, size(simulation.truth.cummulative_landmark_in_FOV,1));
for ii = 1:size(simulation.truth.cummulative_landmark_in_FOV,1)
    true_card(ii) = size(simulation.truth.cummulative_landmark_in_FOV{ii,1},2);
end

est_card = zeros(size(simulation.result,1), size(simulation.truth.cummulative_landmark_in_FOV,1));
ospa = est_card;
cola = est_card;


% Iterate through sim results to find avg errors
for ii = 1:size(simulation.result,1)
    if ii == 1
        slam_pos_error = abs(simulation.truth.pos -...
            simulation.result{ii,1}.filter_est.pos);
        odom_pos_error = abs(simulation.truth.pos -...
            simulation.result{ii,1}.odom_est.pos);

    else
        slam_pos_error = ((slam_pos_error * (ii-1)) + abs(simulation.truth.pos -...
            simulation.result{ii,1}.filter_est.pos))/ii;
        odom_pos_error = ((slam_pos_error * (ii-1)) + abs(simulation.truth.pos -...
            simulation.result{ii,1}.odom_est.pos))/ii;
    end

    for kk = 2:size(true_card,2)
        est_card(ii,kk) = simulation.result{ii,1}.filter_est.map_est{kk,1}.exp_num_landmark;
    end

end

avg_card = mean(est_card,1);
std_card = std(est_card,1);

plot(true_card);

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

%% Mapping error
% Time varying OSPA
ospa_c = 2;
ospa_p = 2;
ospa_vals = zeros(size(time_vec,2),3);
for kk = 2:size(time_vec,2)
    true_map = get_comps(simulation.result{1,1}.truth.cummulative_landmark_in_FOV{1,1},[1,2]);
    est_map = get_comps(simulation.result{1,1}.truth.cummulative_landmark_in_FOV{1,1},[1,2]);
    [ospa_vals(kk,1), ospa_vals(kk,2), ospa_vals(kk,3)] = ospa_dist (true_map,...
        est_map, ospa_c, ospa_p);
end

% COLA of total map
cola_c = ospa_c;
cola_p = ospa_p;

cola_val = zeros(1,3);
[cola_vals(kk,1), cola_vals(kk,2), cola_vals(kk,3)] = cola_dist (true_map,...
    est_map, cola_c, cola_p);

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

figure(4)
subplot (3,1,1);
plot (time_vec, ospa_vals(:,1),'k')
ylabel('OPSA Dist')
ylim([0 ospa_c])
grid on

subplot (3,1,2);
plot (time_vec, ospa_vals(:,2),'k')
ylabel('OPSA Loc')
ylim([0 ospa_c])
grid on

subplot (3,1,3);
plot (time_vec, ospa_vals(:,3),'k')
ylabel('OPSA Card')
ylim([0 ospa_c])
grid on


%% Util functions
function Xc= get_comps(X,c)
    if isempty(X)
        Xc= [];
    else
        Xc= X(c,:);
    end
end