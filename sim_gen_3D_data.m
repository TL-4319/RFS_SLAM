% Simulation. The math is general for 3D but the simulation is limit to 2D
close all;
clear;
clc;

addpath("util/")
rng(69431);
dt = 0.06;
draw = false;
%% Generate landmark map
map_size = 100;
num_landmark = 600;
 landmark_locations = vertcat((rand(1,num_landmark)-0.2) * 2 * map_size ,...
     (rand(1,num_landmark)-0.2) * 2 * map_size, (rand(1,num_landmark)-0.9) * 3);
%landmark_locations = vertcat((rand(1,num_landmark)-0.2) * 2 * map_size ,...
%    (rand(1,num_landmark)-0.2) * 2 * map_size, (rand(1,num_landmark)-0.9) * 0.1);
marker_size = ones(size(landmark_locations,2),1) * 10;

%% Sensor/Robot
% Sensor properties
sensor.HFOV = deg2rad(110);
sensor.VFOV = deg2rad(70);
sensor.max_range = 15;
sensor.min_range = 0.4;
sensor.P_d = 0.9;
sensor.clutter_rate = 2;
sensor.sigma = 0.1;

% Generate trajectory
waypoints = [0,0,0; ... % Initial position
             20, 10, 0;...
             20, 60, 0; ...
             40, 80, 0;...
             50, 90, 0; ...
             90, 80, 0; ...
             100, 100, 0];    % Final position

orientation_wp = quaternion([0,0,0; ...
                          30,0,0;...
                          90,10,0;...
                          70,0,5;...
                          50,0,-10;...
                          -10, 0, 0;...
                          70,0,0],...
                          "eulerd","ZYX","frame");

groundspeed = ones(1,size(waypoints,1)) * 0.7; groundspeed(1) = 0; %Initial zero velocity

[pos, quat, trans_vel, acc_body, acc_world, rot_vel, rot_vel_world, time_vec] = generate_trajectory2(waypoints,...
    orientation_wp, groundspeed, dt);


%% Prealocate truth data
truth.pos = pos;
truth.body_trans_vel = trans_vel;
truth.body_rot_vel = rot_vel;
truth.body_trans_accel = acc_body;
truth.quat = quat;
truth.time_vec = time_vec;
truth.landmark_locations = landmark_locations;
truth.landmark_in_FOV = cell(size(time_vec,2),1);

truth.sensor_params = sensor;

truth.meas_table = cell(size(time_vec,2),1);

if draw
    fig1 = figure(1);
    title ("Sim world")
    fig1.Position=[0,0, 1000, 1000];
    zlim([-4 1])
    % fig2 = figure(2);
    % title("Sensor frame")
    % movegui (fig2, [570,0])
end
% Draw first frame

% Generate measurement sets
[meas, landmark_inFOV] = gen_noisy_3D_meas (pos(:,1), quat(1,1), landmark_locations, sensor);

truth.landmark_in_FOV{1,1} = landmark_inFOV;
truth.cumulative_landmark_in_FOV = truth.landmark_in_FOV;

% Reproject meas into world frame for double checking
meas_world = reproject_meas (pos(:,1), quat(1,1), meas);

truth.meas_table{1,1} = meas;

 if draw
        % Plotting
        figure(1)
        draw_trajectory(pos(:,1), quat(1,1), [0;0;0], 1, 10, 2,'k',false);
        set(gca, 'Zdir', 'reverse')
        set(gca, 'Ydir', 'reverse')
        grid on
        %view([0,90])
        hold on
        scatter3(landmark_locations(1,:),landmark_locations(2,:),landmark_locations(3,:), marker_size,'k')
        scatter3(meas_world(1,:), meas_world(2,:), meas_world(3,:), 'b*')
        hold off
        xlabel("X");
        ylabel("Y");
        zlabel("Z");
        %xlim([-(map_size + 5), (map_size + 5)])
        %ylim([-(map_size + 5), (map_size + 5)])
        axis equal
        indx_name = pad(sprintf('%d',i), 3, 'left','0');
    
        % figure(2)
        % scatter3(meas(1,:), meas(2,:), meas(3,:), 'b*')
        % xlabel("X");
        % ylabel("Y");
        % set(gca, 'Zdir', 'reverse')
        % set(gca, 'Ydir', 'reverse')
        % grid on
        % view([-90,90])
        % xlim([-sensor.max_range, sensor.max_range])
        % ylim([-sensor.max_range, sensor.max_range])
        % hold on
        % plot3 ([0,10],[0,0], [0,0],'r',LineWidth=2)
        % plot3 ([0,0],[0,10], [0,0],'g',LineWidth=2)
        % hold off
        % axis square
        
        map_name = strcat('/home/tuan/Projects/anarsh/visual_RB_PHD_SLAM/map/','001','.png');
        meas_name = strcat('/home/tuan/Projects/anarsh/visual_RB_PHD_SLAM/meas/','001','.png');
        %saveas (fig1,map_name);
        %saveas (fig2, meas_name);
        drawnow
    end

cumulative_landmark = [];

%% Run simulation
for i=2:size(time_vec,2)
    disp(time_vec(i))
    % Generate measurement sets
    [meas, landmark_inFOV] = gen_noisy_3D_meas (pos(:,i), quat(i,:), landmark_locations, sensor);
    
    temp = unique(vertcat(truth.cumulative_landmark_in_FOV{i-1,1}(:,:)', landmark_inFOV'),'rows');

    truth.cumulative_landmark_in_FOV{i,1} = temp';
    
    % Reproject meas into world frame for double checking
    meas_world = reproject_meas (pos(:,i), quat(i,:), meas);
    
    truth.landmark_in_FOV{i,1} = landmark_inFOV;
    truth.meas_table{i,1} = meas;

    if draw
        % Plotting
        figure(1)
        draw_trajectory(pos(:,i), quat(i,:), pos(:,1:i-1), 1, 10, 2,'k',false);
        set(gca, 'Zdir', 'reverse')
        set(gca, 'Ydir', 'reverse')
        grid on
        %view([0,40])
        hold on
        scatter3(landmark_locations(1,:),landmark_locations(2,:),landmark_locations(3,:),ones(size(landmark_locations,2),1) * 10,'k')
        marker_size = ones(size(truth.cumulative_landmark_in_FOV{i,1},2),1) * 10;
        scatter3(truth.cumulative_landmark_in_FOV{i,1}(1,:),truth.cumulative_landmark_in_FOV{i,1}(2,:),truth.cumulative_landmark_in_FOV{i,1}(3,:), marker_size,'magenta')
        scatter3(meas_world(1,:), meas_world(2,:), meas_world(3,:), 'b*')
        hold off
        xlabel("X");
        ylabel("Y");
        zlabel("Z");
        
        axis equal
        indx_name = pad(sprintf('%d',i), 3, 'left','0');
        zlim([-4 1])
        drawnow
    end
end

figure(1)
draw_trajectory(pos(:,i), quat(i,:), pos(:,1:i-1), 1, 10, 2,'k',false);
set(gca, 'Zdir', 'reverse')
set(gca, 'Ydir', 'reverse')
grid on
%view([0,40])
hold on
%scatter3(landmark_locations(1,:),landmark_locations(2,:),landmark_locations(3,:),ones(size(landmark_locations,2),1) * 10,'k')
marker_size = ones(size(truth.cumulative_landmark_in_FOV{end,1},2),1) * 10;
scatter3(truth.cumulative_landmark_in_FOV{end,1}(1,:),truth.cumulative_landmark_in_FOV{end,1}(2,:),truth.cumulative_landmark_in_FOV{end,1}(3,:), marker_size,'magenta')
%scatter3(meas_world(1,:), meas_world(2,:), meas_world(3,:), 'b*')
hold off
xlabel("X");
ylabel("Y");
zlabel("Z");

axis equal
indx_name = pad(sprintf('%d',i), 3, 'left','0');
zlim([-4 1])




