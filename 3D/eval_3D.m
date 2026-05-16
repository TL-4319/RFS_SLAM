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
% [file,location] = uigetfile;
% load (strcat(location,file));

%load ('sim_result/sim-20250723-1109-pd05.mat');
load('sim_result/sim-20250729-1655-nominal.mat');
%load('sim_result/sim-20250115-1225-phd-slam-100.mat');
name_to_save = "run1_std";
%% 
time_vec = simulation.truth.time_vec;
dt = time_vec(2) - time_vec(1);
sensor_time_vec = simulation.truth.sensor_time_vec;
sensor_dt = sensor_time_vec(2) - sensor_time_vec(1);

% Pre allocate traj metrics
slam_posx_error = zeros(size(simulation.result,1),size(simulation.truth.pos,2));
slam_posx_error = zeros(1,size(simulation.truth.pos,2));
slam_posy_error = slam_posx_error;
slam_posz_error = slam_posx_error;
slam_eulx_error = slam_posx_error;
slam_euly_error = slam_posx_error;
slam_eulz_error = slam_posx_error;

odom_posx_error = slam_posx_error;
odom_posy_error = slam_posx_error;
odom_posz_error = slam_posx_error;
odom_eulx_error = slam_posx_error;
odom_euly_error = slam_posx_error;
odom_eulz_error = slam_posx_error;

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

% Compute time
compute_time = zeros(100, size(time_vec,2));

% Iterate through sim results to find avg errors
%for ii = 1:size(simulation.result,1)
for ii = 1:100
    disp(ii)
    slam_pos_error = abs(simulation.truth.pos -...
            simulation.result{ii,1}.filter_est.pos);
    odom_pos_error = abs(simulation.truth.pos -...
            simulation.result{ii,1}.odom_est.pos);
    
    true_eul = transpose(quat2eul(simulation.truth.quat));
    slam_eul = transpose(quat2eul(simulation.result{ii,1}.filter_est.quat));
    odom_eul = transpose(quat2eul(simulation.result{ii,1}.odom_est.quat));

    slam_eul_error = abs(true_eul - slam_eul);
    odom_eul_error = abs(true_eul - odom_eul);

    slam_posx_error(ii,:) = slam_pos_error(1,:);
    slam_posy_error(ii,:) = slam_pos_error(2,:);
    slam_posz_error(ii,:) = slam_pos_error(3,:);

    odom_posx_error(ii,:) = odom_pos_error(1,:);
    odom_posy_error(ii,:) = odom_pos_error(2,:);
    odom_posz_error(ii,:) = odom_pos_error(3,:);

    slam_eulx_error(ii,:) = slam_eul_error(3,:);
    slam_euly_error(ii,:) = slam_eul_error(2,:);
    slam_eulz_error(ii,:) = slam_eul_error(1,:);

    odom_eulx_error(ii,:) = odom_eul_error(3,:);
    odom_euly_error(ii,:) = odom_eul_error(2,:);
    odom_eulz_error(ii,:) = odom_eul_error(1,:);    
    
    compute_time(ii,:) = simulation.result{ii,1}.filter_est.compute_time';
end
avg_compute_time = mean(compute_time,1);
avg_compute_time_meas = avg_compute_time(avg_compute_time>0.1);
mean_compute_time = mean(avg_compute_time_meas)
std_compute_time = std(avg_compute_time_meas)



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

dist_travel = simulation.truth.pos;
dist_travel(:,2:end) = simulation.truth.pos(:,2:end) - simulation.truth.pos(:,1:end-1);
dist_travel = vecnorm(dist_travel);
dist_travel = cumsum(dist_travel);

error.filter_pos_error = slam_pos_error;
error.filter_eul_error = slam_eul_error;
error.odom_pos_error = odom_pos_error;
error.odom_eul_error = odom_eul_error;
error.avg_card = avg_card;
error.std_card = std_card;
error.avg_ospa = vertcat(avg_ospa_1, avg_ospa_2, avg_ospa_3);
error.avg_cola = vertcat(avg_cola_1, avg_cola_2, avg_cola_3);
error.time_vec = time_vec;
error.dist_travel = dist_travel;
error.compute_time = avg_compute_time;

slam_avg_posx_err = mean(slam_posx_error,1);
slam_std_posx_err = std(slam_posx_error,1);
slam_avg_posy_err = mean(slam_posy_error,1);
slam_std_posy_err = std(slam_posy_error,1);
slam_avg_posz_err = mean(slam_posz_error,1);
slam_std_posz_err = std(slam_posz_error,1);

slam_pos_error = (slam_posx_error(:,end).^2 + slam_posy_error(:,end).^2 + ...
    slam_posz_error(:,end).^2).^0.5;
mean_pos_error = mean(slam_pos_error)
std_pos_error = std(slam_pos_error)

odom_avg_posx_err = mean(odom_posx_error,1);
odom_std_posx_err = std(odom_posx_error,1);
odom_avg_posy_err = mean(odom_posy_error,1);
odom_std_posy_err = std(odom_posy_error,1);
odom_avg_posz_err = mean(odom_posz_error,1);
odom_std_posz_err = std(odom_posz_error,1);

odom_pos_error = (odom_posx_error(:,end).^2 + odom_posy_error(:,end).^2 + ...
    odom_posz_error(:,end).^2).^0.5;

slam_avg_eulx_err = mean(slam_eulx_error,1);
slam_std_eulx_err = std(slam_eulx_error,1);
slam_avg_euly_err = mean(slam_euly_error,1);
slam_std_euly_err = std(slam_euly_error,1);
slam_avg_eulz_err = mean(slam_eulz_error,1);
slam_std_eulz_err = std(slam_eulz_error,1);

odom_avg_eulx_err = mean(odom_eulx_error,1);
odom_std_eulx_err = std(odom_eulx_error,1);
odom_avg_euly_err = mean(odom_euly_error,1);
odom_std_euly_err = std(odom_euly_error,1);
odom_avg_eulz_err = mean(odom_eulz_error,1);
odom_std_eulz_err = std(odom_eulz_error,1);

odom_error = (odom_avg_posx_err.^2 + odom_avg_posy_err.^2 +...
    odom_avg_posz_err.^2).^0.5;


pos_error = (slam_avg_posx_err.^2 + slam_avg_posy_err.^2 +...
    slam_avg_posz_err.^2).^0.5;
slam_per = pos_error(end)/dist_travel(end)
odom_per = odom_error(end)/dist_travel(end)

figure(1)
subplot (3,1,1)
plot (dist_travel, slam_avg_posx_err,'k','DisplayName','PHD-SLAM','LineWidth',2)
hold on 
plot (dist_travel, slam_avg_posx_err+slam_std_posx_err,'k--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, slam_avg_posx_err-slam_std_posx_err,'k--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, odom_avg_posx_err,'r','DisplayName','Odometry','LineWidth',2)
plot (dist_travel, odom_avg_posx_err+odom_std_posx_err,'r--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, odom_avg_posx_err-odom_std_posx_err,'r--','LineWidth',1,'HandleVisibility','off')
xlabel("Dist travelled (m)")
ylabel("X error (m)")
grid on
xlim([0,dist_travel(end)])
% for ii=1:size(slam_posx_error,1)
% plot (dist_travel, slam_posx_error(ii,:),'Color',[0,0,0,0.05],'LineWidth',1,'HandleVisibility','off')
% plot (dist_travel, odom_posx_error(ii,:),'Color',[1,0,0,0.05],'LineWidth',1,'HandleVisibility','off')
% end
legend('Location','northwest')
modify_figure(15)

subplot (3,1,2)
plot (dist_travel, slam_avg_posy_err,'k','DisplayName','PHD-SLAM','LineWidth',2)
hold on 
plot (dist_travel, slam_avg_posy_err+slam_std_posy_err,'k--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, slam_avg_posy_err-slam_std_posy_err,'k--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, odom_avg_posy_err,'r','DisplayName','Odometry','LineWidth',2)
plot (dist_travel, odom_avg_posy_err+odom_std_posy_err,'r--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, odom_avg_posy_err-odom_std_posy_err,'r--','LineWidth',1,'HandleVisibility','off')
xlabel("Dist travelled (m)")
ylabel("Y error (m)")
grid on
xlim([0,dist_travel(end)])
modify_figure(15)
% for ii=1:size(slam_posx_error,1)
% plot (dist_travel, slam_posy_error(ii,:),'Color',[0,0,0,0.05],'LineWidth',1,'HandleVisibility','off')
% plot (dist_travel, odom_posy_error(ii,:),'Color',[1,0,0,0.05],'LineWidth',1,'HandleVisibility','off')
% end

subplot (3,1,3)
plot (dist_travel, slam_avg_posz_err,'k','DisplayName','PHD-SLAM','LineWidth',2)
hold on 
plot (dist_travel, slam_avg_posz_err+slam_std_posz_err,'k--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, slam_avg_posz_err-slam_std_posz_err,'k--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, odom_avg_posz_err,'r','DisplayName','Odometry','LineWidth',2)
plot (dist_travel, odom_avg_posz_err+odom_std_posz_err,'r--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, odom_avg_posz_err-odom_std_posz_err,'r--','LineWidth',1,'HandleVisibility','off')
xlabel("Dist travelled (m)")
ylabel("Z error (m)")
grid on
xlim([0,dist_travel(end)])
% for ii=1:size(slam_posx_error,1)
% plot (dist_travel, slam_posz_error(ii,:),'Color',[0,0,0,0.05],'LineWidth',1,'HandleVisibility','off')
% plot (dist_travel, odom_posz_error(ii,:),'Color',[1,0,0,0.05],'LineWidth',1,'HandleVisibility','off')
% end
modify_figure(15)
saveas(gcf,strcat(name_to_save,"_pos.png"))

figure(2)
subplot (3,1,3)
plot (dist_travel, slam_avg_eulx_err * 57.296,'k','DisplayName','PHD-SLAM','LineWidth',2)
hold on 
plot (dist_travel, odom_avg_eulx_err* 57.296,'r','DisplayName','Odometry','LineWidth',2)
plot (dist_travel, (slam_avg_eulx_err+slam_std_eulx_err)*57.296,'k--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, (slam_avg_eulx_err-slam_std_eulx_err)*57.296,'k--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, (odom_avg_eulx_err+odom_std_eulx_err)*57.296,'r--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, (odom_avg_eulx_err-odom_std_eulx_err)*57.296,'r--','LineWidth',1,'HandleVisibility','off')
xlabel("Dist travelled (m)")
ylabel("X error (^o)")
grid on

xlim([0,dist_travel(end)])
% for ii=1:size(slam_posx_error,1)
% plot (dist_travel, slam_posx_error(ii,:),'Color',[0,0,0,0.05],'LineWidth',1,'HandleVisibility','off')
% plot (dist_travel, odom_posx_error(ii,:),'Color',[1,0,0,0.05],'LineWidth',1,'HandleVisibility','off')
% end

modify_figure(15)

subplot (3,1,2)
plot (dist_travel, slam_avg_euly_err* 57.296,'k','DisplayName','PHD-SLAM','LineWidth',2)
hold on 
plot (dist_travel, odom_avg_euly_err* 57.296,'r','DisplayName','Odometry','LineWidth',2)
plot (dist_travel, (slam_avg_euly_err+slam_std_euly_err)*57.296,'k--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, (slam_avg_euly_err-slam_std_euly_err)*57.296,'k--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, (odom_avg_euly_err+odom_std_euly_err)*57.296,'r--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, (odom_avg_euly_err-odom_std_euly_err)*57.296,'r--','LineWidth',1,'HandleVisibility','off')
xlabel("Dist travelled (m)")
ylabel("Y error (^o)")
grid on

xlim([0,dist_travel(end)])
modify_figure(15)
% for ii=1:size(slam_posx_error,1)
% plot (dist_travel, slam_posy_error(ii,:),'Color',[0,0,0,0.05],'LineWidth',1,'HandleVisibility','off')
% plot (dist_travel, odom_posy_error(ii,:),'Color',[1,0,0,0.05],'LineWidth',1,'HandleVisibility','off')
% end

subplot (3,1,1)
plot (dist_travel, slam_avg_eulz_err* 57.296,'k','DisplayName','PHD-SLAM','LineWidth',2)
hold on 
plot (dist_travel, odom_avg_eulz_err* 57.296,'r','DisplayName','Odometry','LineWidth',2)
plot (dist_travel, (slam_avg_eulz_err+slam_std_eulz_err)*57.296,'k--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, (slam_avg_eulz_err-slam_std_eulz_err)*57.296,'k--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, (odom_avg_eulz_err+odom_std_eulz_err)*57.296,'r--','LineWidth',1,'HandleVisibility','off')
plot (dist_travel, (odom_avg_eulz_err-odom_std_eulz_err)*57.296,'r--','LineWidth',1,'HandleVisibility','off')
xlabel("Dist travelled (m)")
ylabel("Z error (^o)")
grid on
xlim([0,dist_travel(end)])
% for ii=1:size(slam_posx_error,1)
% plot (dist_travel, slam_posz_error(ii,:),'Color',[0,0,0,0.05],'LineWidth',1,'HandleVisibility','off')
% plot (dist_travel, odom_posz_error(ii,:),'Color',[1,0,0,0.05],'LineWidth',1,'HandleVisibility','off')
% end
legend('Location','northwest')
modify_figure(15)
saveas(gcf,strcat(name_to_save,"_euler.png"))

figure(3)
bins = 0:0.2:4;
histogram(slam_pos_error, bins,'FaceColor','k','DisplayName','PHD-SLAM')
hold on
histogram(odom_pos_error, bins,'FaceColor','r','DisplayName','Odometry')
ylabel("Count")
xlabel("Final pos error (m)")
modify_figure(15)
legend
saveas(gcf,strcat(name_to_save,"_pos_dist.png"))


%% Util functions
function Xc= get_comps(X,c)
    if isempty(X)
        Xc= [];
    else
        Xc= X(c,:);
    end
end

