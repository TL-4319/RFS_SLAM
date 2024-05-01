% Simulation. The math is general for 3D but the simulation is limit to 2D
close all;
clear;
clc;

addpath("util/")
rng(69431);
dt = 0.2;
draw = false;

image_rate = 15;
imu_rate = 100;

image_dt = 1/image_rate;
imu_dt = 1/imu_rate;

%% Generate landmark map
map_size = 100;
num_landmark = 3000;
landmark_locations = vertcat((rand(1,num_landmark)-0.2) * 2 * map_size ,...
    (rand(1,num_landmark)-0.2) * 2 * map_size, (rand(1,num_landmark)-0.9) * 3);
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
             50, 80, 0;...
             90, 80, 0;...
             100,100,0];    % Final position

orientation_wp = quaternion([0,0,0; ...
                          30,10,0;...
                          90,0,0;...
                          70,0,0;...
                          50,0,0;...
                          -10,0,0;...
                          50,0,0],...
                          "eulerd","ZYX","frame");

groundspeed = ones(1,size(waypoints,1)) * 0.8; groundspeed(1) = 0; %Initial zero velocity

[pos, quat, trans_vel, vel_world, acc_body, acc_world, rot_vel_body, rot_vel_world, imu_time_vec] = generate_trajectory2(waypoints,...
    orientation_wp, groundspeed, imu_dt);


%% Generate img timestamps
has_img = zeros(size(imu_time_vec));
has_img(1) = 1;
img_time_vec = 0;
cur_time = 0;
last_time = 0;
for t = 2:size(imu_time_vec,2)
    cur_time = cur_time + imu_dt;
    delta = cur_time-last_time;
    if abs(delta - image_dt) <= 0.01
        img_time_vec = horzcat(img_time_vec, cur_time);
        has_img(t) = 1;
        last_time = cur_time;
    end
end


%% Prealocate truth data
truth.pos = pos;
truth.body_trans_vel = trans_vel;
truth.body_rot_vel = rot_vel_body;
truth.world_accel = acc_world;
truth.world_vel = vel_world;
truth.world_rot_vel = rot_vel_world;
truth.quat = quat;
truth.imu_time_vec = imu_time_vec;
truth.landmark_locations = landmark_locations;
truth.landmark_in_FOV = cell(size(img_time_vec,2),1);
truth.img_time_vec = img_time_vec;
truth.has_img = has_img;
truth.img_to_imu_map = find(has_img); %Mapping from img_time_vec corresponding IMU time vec
truth.sensor_params = sensor;

truth.meas_table = cell(size(img_time_vec,2),1);

if draw
    fig1 = figure(1);
    title ("Sim world")
    fig1.Position=[0,0, 1000, 1000];
    
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
        
        
        %saveas (fig1,map_name);
        %saveas (fig2, meas_name);
        drawnow
    end

%% Run measurement simulation
k = 2;

for i=2:size(imu_time_vec,2)
    disp(imu_time_vec(i))
    
    % Only generate measurement at correct time
    if has_img(i) == 1
        % Generate measurement sets
        [meas, landmark_inFOV] = gen_noisy_3D_meas (pos(:,i), quat(i,:), landmark_locations, sensor);
        
        temp = unique(vertcat(truth.cumulative_landmark_in_FOV{k-1,1}(:,:)',...
            landmark_inFOV'),'rows');

        truth.cumulative_landmark_in_FOV{k,1} = temp';
        % Reproject meas into world frame for double checking
        meas_world = reproject_meas (pos(:,i), quat(i,:), meas);
        
        truth.landmark_in_FOV{k,1} = landmark_inFOV;
        truth.meas_table{k,1} = meas;
        dbg_string = sprintf('imu time: %f ; img time: %f',imu_time_vec(i), img_time_vec(k));
        k = k + 1;
        disp (dbg_string);
        %% Visualize simulation
        if draw
        % Plotting
        figure(1)
        draw_trajectory(pos(:,i), quat(i,:), pos(:,1:i-1), 1, 10, 2,'k',false);
        set(gca, 'Zdir', 'reverse')
        set(gca, 'Ydir', 'reverse')
        grid on
        %view([0,40])
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
        
        %map_name = strcat('/home/tuan/Projects/anarsh/visual_RB_PHD_SLAM/map/',indx_name,'.png');
        %meas_name = strcat('/home/tuan/Projects/anarsh/visual_RB_PHD_SLAM/meas/',indx_name,'.png');
        %saveas (fig1,map_name);
        %saveas (fig2, meas_name);
        drawnow
    end
    end


    
end



