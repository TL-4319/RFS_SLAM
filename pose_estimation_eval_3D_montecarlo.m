close all
clear
clc

load ("monte_res_100.mat")

num_run = size(monte_result,1);

metrics = zeros(num_run,14);

addpath("util/");

%%



for ii = 1:num_run
    simulation = monte_result{ii};
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

    %% Propagate only odometry
    odom_pos = simulation.truth.pos;
    odom_quat = simulation.truth.quat;
    odom_body_vel = simulation.odom.body_trans_vel;
    odom_body_rot_vel = simulation.odom.body_rot_vel;
    for tt = 2:size(odom_pos,2)
        [odom_pos(:,tt),odom_quat(tt,:)] = propagate_state(odom_pos(:,tt-1), odom_quat(tt-1,:), ...
            odom_body_vel(:,tt), odom_body_rot_vel(:,tt), dt);
    end
    odom_euler = transpose(quat2eul(odom_quat));

    %% Error calc
    pos_error = est_pos - true_pos;
    dist_error = vecnorm(pos_error);
    
    euler_error = est_euler - true_euler;

    % Odometry only error
    odom_pos_error = odom_pos - true_pos;
    odom_dis_error = vecnorm(odom_pos_error);
    
    odom_euler_error = odom_euler - true_euler;

    %% RMSE error
    pos_rmse = sum(pos_error.^2,2)./size(pos_error,2);
    pos_rmse = pos_rmse.^2;

    euler_rmse = sum(euler_error.^2,2)./size(euler_error,2);
    euler_rmse = euler_rmse.^2;

    odom_pos_rmse = sum(odom_pos_error.^2,2)./size(odom_pos_error,2);
    odom_pos_rmse = odom_pos_rmse.^2;

    odom_euler_rmse = sum(odom_euler_error.^2,2)./size(odom_euler_error,2);
    odom_euler_rmse = odom_euler_rmse.^2;
    
    metrics(ii,1:3) = pos_rmse';
    metrics(ii,4:6) = odom_pos_rmse';
    metrics(ii,7:9) = euler_rmse';
    metrics(ii,10:12) = odom_euler_rmse';
    metrics(ii,13) = dist_error(end);
    metrics(ii,14) = odom_dis_error(end);

    %% Average error over trajectory

    if ii == 1
        avg_pos_error = abs(pos_error);
        avg_dis_error = dist_error;
        avg_euler_error = abs(euler_error);

        avg_odom_pos_error = abs(odom_pos_error);
        avg_odom_dis_error = odom_dis_error;
        avg_odom_euler_error = abs(odom_euler_error);

        avg_pos = est_pos;
        avg_euler = est_euler;
        avg_odom_pos = odom_pos;
        avg_odom_euler = odom_euler;
    else
        avg_pos_error = (avg_pos_error * (ii-1) + abs(pos_error)) / ii;
        avg_dis_error = (avg_dis_error * (ii-1) + dist_error) / ii;
        avg_euler_error = (avg_euler_error * (ii-1) + abs(euler_error)) / ii;

        avg_odom_pos_error = (avg_odom_pos_error * (ii-1) + abs(odom_pos_error)) / ii;
        avg_odom_dis_error = (avg_odom_dis_error * (ii-1) + odom_dis_error) / ii;
        avg_odom_euler_error = (avg_odom_euler_error * (ii-1) + abs(odom_euler_error)) / ii;

        avg_pos = (avg_pos * (ii-1) + est_pos) / ii;
        avg_euler = (avg_euler * (ii-1) + est_euler) / ii;

        avg_odom_pos = (avg_odom_pos * (ii-1) + odom_pos) / ii;
        avg_odom_euler = (avg_odom_euler * (ii-1) + odom_euler) / ii;
    end

    disp(ii)

end

%% Plotting
close all

font_sz = 15;

figure(1)
subplot (3,1,1)
plot (time_vec, abs(avg_pos_error(1,:)),'DisplayName','PHD-SLAM',LineWidth=2)
hold on 
plot (time_vec, abs(avg_odom_pos_error(1,:)),'DisplayName','Odometry',LineWidth=2)
xlabel("Time (s)","FontSize",font_sz)
ylabel("X error (m)","FontSize",font_sz)
grid on
title("Translational error","FontSize",font_sz)
legend ("FontSize",font_sz)
modify_figure(font_sz)

subplot (3,1,2)
plot (time_vec, abs(avg_pos_error(2,:)),'DisplayName','PHD-SLAM',LineWidth=2)
hold on
plot (time_vec, abs(avg_odom_pos_error(2,:)),'DisplayName','Odometry',LineWidth=2)
xlabel("Time (s)","FontSize",font_sz)
ylabel("Y error (m)","FontSize",font_sz)
grid on
modify_figure(font_sz)

subplot (3,1,3)
plot (time_vec, abs(avg_pos_error(3,:)),'DisplayName','PHD-SLAM',LineWidth=2)
hold on
plot (time_vec, abs(avg_odom_pos_error(3,:)),'DisplayName','Odometry',LineWidth=2)
xlabel("Time (s)","FontSize",font_sz)
ylabel("Z error (m)","FontSize",font_sz)
grid on
modify_figure(font_sz)

figure(2)
plot (time_vec, avg_dis_error,'DisplayName','PHD-SLAM',LineWidth=2)
hold on
plot (time_vec, avg_odom_dis_error,'DisplayName','Odometry',LineWidth=2)
xlabel("Time (s)","FontSize",font_sz)
ylabel("Error (m)","FontSize",font_sz)
grid on
title("Distance error","FontSize",font_sz)
legend("FontSize",font_sz)
modify_figure(font_sz)

figure(3)
subplot(3,1,1)
plot (time_vec, abs(avg_euler_error(3,:)) * 180/pi,'DisplayName','PHD-SLAM',LineWidth=2)
hold on
plot (time_vec, abs(avg_odom_euler_error(3,:)) * 180/pi, 'DisplayName','Odometry',LineWidth=2)
xlabel("Time (s)","FontSize",font_sz)
ylabel("Roll error (deg)","FontSize",font_sz)
grid on
title("Rotational error","FontSize",font_sz)
legend ("FontSize",font_sz)
modify_figure(font_sz)

subplot(3,1,2)
plot (time_vec, abs(avg_euler_error(2,:)) * 180/pi,'DisplayName','PHD-SLAM',LineWidth=2)
hold on
plot (time_vec, abs(avg_odom_euler_error(2,:)) * 180/pi, 'DisplayName','Odometry',LineWidth=2)
xlabel("Time (s)","FontSize",font_sz)
ylabel("Pitch error (deg)","FontSize",font_sz)
grid on
modify_figure(font_sz)

subplot(3,1,3)
plot (time_vec, abs(avg_euler_error(1,:)) * 180/pi,'DisplayName','PHD-SLAM',LineWidth=2)
hold on
plot (time_vec, abs(avg_odom_euler_error(1,:)) * 180/pi, 'DisplayName','Odometry',LineWidth=2)
xlabel("Time (s)","FontSize",font_sz)
ylabel("Yaw error (deg)","FontSize",font_sz)
grid on
modify_figure(font_sz)

function modify_figure(fontsize)
grid on; grid minor;
ax=gca;
set(ax,'FontName','Times','Fontsize',fontsize) %,'FontWeight','bold');
box on
set(gcf,'color',[1 1 1]) %to make the backgroung white
set(gca, 'FontName', 'Arial')
end