close all;
clear; 
clc;

addpath('../util/')

ospa_c = 2;
ospa_p = 2;
cola_c = 2;
cola_p = 2;

%% Select simulation
% Parse simulation
[file,location] = uigetfile;
load (strcat(location,file));


%% 
time_vec = simulation.truth.time_vec;
dt = time_vec(2) - time_vec(1);
sensor_time_vec = simulation.truth.sensor_time_vec;
sensor_dt = sensor_time_vec(2) - sensor_time_vec(1);

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

% Cardinality
est_card = zeros(size(simulation.result,1), size(simulation.truth.cummulative_landmark_in_FOV,1));

% Time varying OSPA
ospa_1 = est_card;
ospa_2 = est_card;
ospa_3 = est_card;

% Total COLA metric
cola_1 = zeros(size(simulation.result,1),1);
cola_2 = cola_1;
cola_3 = cola_1;


% Iterate through sim results to find avg errors
for ii = 1:size(simulation.result,1)
    if ii == 1
        slam_pos_error = abs(simulation.truth.pos -...
            simulation.result{ii,1}.filter_est.pos);
        odom_pos_error = abs(simulation.truth.pos -...
            simulation.result{ii,1}.odom_est.pos);

        true_eul = transpose(quat2eul(simulation.truth.quat));
        slam_eul = transpose(quat2eul(simulation.result{ii,1}.filter_est.quat));
        odom_eul = transpose(quat2eul(simulation.result{ii,1}.odom_est.quat));

        slam_eul_error = abs(true_eul - slam_eul);
        odom_eul_error = abs(true_eul - odom_eul);

    else
        slam_pos_error = ((slam_pos_error * (ii-1)) + abs(simulation.truth.pos -...
            simulation.result{ii,1}.filter_est.pos))/ii;
        odom_pos_error = ((slam_pos_error * (ii-1)) + abs(simulation.truth.pos -...
            simulation.result{ii,1}.odom_est.pos))/ii;

        true_eul = transpose(quat2eul(simulation.truth.quat));
        slam_eul = transpose(quat2eul(simulation.result{ii,1}.filter_est.quat));
        odom_eul = transpose(quat2eul(simulation.result{ii,1}.odom_est.quat));

        slam_eul_error = ((slam_eul_error * (ii-1)) + abs(true_eul -...
           slam_eul))/ii;
        odom_eul_error = ((odom_eul_error * (ii-1)) + abs(true_eul -...
           odom_eul))/ii;
    end
    
    % Time dependent error
    for kk = 2:size(sensor_time_vec,2)
        est_card(ii,kk) = simulation.result{ii,1}.filter_est.map_est{kk,1}.exp_num_landmark;

        % Mapping error
        if ii == 1 
            % This only needs to be done once
            true_map = get_comps(simulation.truth.cummulative_landmark_in_FOV{kk,1},[1,2,3]);
        end
        est_map = get_comps(simulation.result{ii,1}.filter_est.map_est{kk,1}.feature_pos,[1,2,3]);

        [ospa_1(ii,kk), ospa_2(ii,kk), ospa_3(ii,kk)] = ospa_dist (true_map,...
        est_map, ospa_c, ospa_p);
    end

    [cola_1(ii), cola_2(ii), cola_3(ii)] = cola_dist (true_map,...
    est_map, cola_c, cola_p);

end

% Card statistics
avg_card = mean(est_card,1);
std_card = std(est_card,1);

% OSPA statistics
avg_ospa_1 = mean(ospa_1,1); 
avg_ospa_2 = mean(ospa_2,1);
avg_ospa_3 = mean(ospa_3,1);

% COLA statistics
avg_cola_1 = mean(cola_1,1);
avg_cola_2 = mean(cola_2,1);
avg_cola_3 = mean(cola_3,1);


% % Calculate distance travelled for relative metric
% dist_travel = true_pos;
% dist_travel(:,2:end) = true_pos(:,2:end) - true_pos(:,1:end-1);
% dist_travel = vecnorm(dist_travel);
% dist_travel = cumsum(dist_travel);
% 
% %% Error calc
% pos_error = est_pos - true_pos;
% dist_error = vecnorm(pos_error);
% 
% euler_error = est_euler - true_euler;
% 
% rel_trans_error = dist_error./dist_travel;
% rel_trans_error(1) = 0;
% 
% rel_euler_error = euler_error;
% rel_euler_error(1,:) = euler_error(1,:) ./ dist_travel;
% rel_euler_error(2,:) = euler_error(2,:) ./ dist_travel;
% rel_euler_error(3,:) = euler_error(3,:) ./ dist_travel;
% rel_euler_error(:,1) = [0;0;0];
% 
% % Odometry only error
% odom_pos_error = odom_pos - true_pos;
% odom_dis_error = vecnorm(odom_pos_error);
% 
% odom_euler_error = odom_euler - true_euler;

%% Plot
figure(1)
subplot (3,1,1)
plot (time_vec, slam_pos_error(1,:),'DisplayName',simulation.type)
hold on 
plot (time_vec, odom_pos_error(1,:),'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("X error")
legend

subplot (3,1,2)
plot (time_vec, slam_pos_error(2,:),'DisplayName',simulation.type)
hold on
plot (time_vec, odom_pos_error(2,:),'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Y error")

subplot (3,1,3)
plot (time_vec, slam_pos_error(3,:),'DisplayName',simulation.type)
hold on
plot (time_vec, odom_pos_error(3,:),'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Z error")

figure(2)
subplot (3,1,1)
plot (time_vec, slam_eul_error(1,:)*pi/180,'DisplayName',simulation.type)
hold on 
plot (time_vec, odom_eul_error(1,:)*pi/180,'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("X error")
legend

subplot (3,1,2)
plot (time_vec, slam_eul_error(2,:)*pi/180,'DisplayName',simulation.type)
hold on
plot (time_vec, odom_eul_error(2,:)*pi/180,'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Y error")

subplot (3,1,3)
plot (time_vec, slam_eul_error(3,:)*pi/180,'DisplayName',simulation.type)
hold on
plot (time_vec, odom_eul_error(3,:)*pi/180,'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Y error")

figure(3)
subplot (3,1,1);
plot (sensor_time_vec, avg_ospa_1,'k')
ylabel('OPSA Dist')
ylim([0 ospa_c])
grid on

subplot (3,1,2);
plot (sensor_time_vec, avg_ospa_2,'k')
ylabel('OPSA Loc')
ylim([0 ospa_c])
grid on

subplot (3,1,3);
plot (sensor_time_vec, avg_ospa_3,'k')
ylabel('OPSA Card')
ylim([0 ospa_c])
grid on

figure(4)
plot(sensor_time_vec,true_card,'r','DisplayName','Truth');
hold on
plot (sensor_time_vec,avg_card,'k','DisplayName','Est mean')
plot (sensor_time_vec,avg_card + std_card, 'k--','DisplayName','Est std')
plot (sensor_time_vec,avg_card - std_card, 'k--','HandleVisibility','off')
grid on
legend

%% Util functions
function Xc= get_comps(X,c)
    if isempty(X)
        Xc= [];
    else
        Xc= X(c,:);
    end
end

