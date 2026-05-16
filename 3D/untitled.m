close all
clear
clc

load ("sim_result/sim-20241230-1509-phd-slam-error.mat")

%%

figure(1)
subplot (3,1,1)
plot (error.dist_travel, error.slam_pos_error(1,:),'DisplayName',simulation.type)
hold on 
plot (error.dist_travel, odom_pos_error(1,:),'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("X error")
legend

subplot (3,1,2)
plot (error.dist_travel, slam_pos_error(2,:),'DisplayName',simulation.type)
hold on
plot (error.dist_travel, odom_pos_error(2,:),'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Y error")

subplot (3,1,3)
plot (error.dist_travel, slam_pos_error(3,:),'DisplayName',simulation.type)
hold on
plot (error.dist_travel, odom_pos_error(3,:),'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Z error")

figure(2)
subplot (3,1,1)
plot (error.dist_travel, slam_eul_error(1,:)*pi/180,'DisplayName',simulation.type)
hold on 
plot (error.dist_travel, odom_eul_error(1,:)*pi/180,'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("X error")
legend

subplot (3,1,2)
plot (error.dist_travel, slam_eul_error(2,:)*pi/180,'DisplayName',simulation.type)
hold on
plot (error.dist_travel, odom_eul_error(2,:)*pi/180,'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Y error")

subplot (3,1,3)
plot (error.dist_travel, slam_eul_error(3,:)*pi/180,'DisplayName',simulation.type)
hold on
plot (error.dist_travel, odom_eul_error(3,:)*pi/180,'DisplayName','Odometry')
xlabel("Time (s)")
ylabel("error (m)")
grid on
title("Y error")

figure(3)
subplot (3,1,1);
plot (1:size(), avg_ospa_1,'k')
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