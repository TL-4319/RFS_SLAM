% Simulation. The math is general for 3D but the simulation is limit to 2D
close all;
clear;
clc;

addpath("util/")
rng(69431);
dt = 0.2;
time_vec = 0:dt:160;
draw = true;
%% Generate landmark map
map_size = 150;
num_landmark = 3000;
landmark_locations = (rand(num_landmark, 3) - 0.2) * 2 * map_size;
landmark_locations(:,3) = zeros(1,size(landmark_locations,1));
landmark_locations = landmark_locations';

%% Sensor/Robot
% Sensor properties
sensor.HFOV = deg2rad(100);
sensor.max_range = 20;
sensor.min_range = 0.4;
sensor.P_d = 0.9;
sensor.clutter_rate = 2;
sensor.sigma = 0.1;

% Generate trajectory
waypoints = [0,0,0; ... % Initial position
             20, 10, 0;...
             20, 60, 0; ...
             40, 80, 0
             120,120,0];    % Final position

orientation_wp = quaternion([0,0,0; ...
                          30,0,0;...
                          90,0,0;...
                          70,0,0;...
                          50,0,0],...
                          "eulerd","ZYX","frame");

groundspeed = ones(1,size(waypoints,1)) * 1; groundspeed(1) = 0; %Initial zero velocity


[pos, quat, trans_vel, acc_body, rot_vel, time_vec] = generate_trajectory(waypoints,...
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
    
    % fig2 = figure(2);
    % title("Sensor frame")
    % movegui (fig2, [570,0])
end
% Draw first frame

% Generate measurement sets
[meas, landmark_inFOV] = gen_noisy_meas (pos(:,1), quat(1,1), landmark_locations, sensor);

truth.landmark_in_FOV{1,1} = landmark_inFOV;

% Reproject meas into world frame for double checking
meas_world = reproject_meas (pos(:,1), quat(1,1), meas);

truth.meas_table{1,1} = meas;

 if draw
        % Plotting
        figure(1)
        draw_trajectory(pos, quat, [0;0;0], 1, 10, 2,'k',false);
        set(gca, 'Zdir', 'reverse')
        set(gca, 'Ydir', 'reverse')
        grid on
        view([0,90])
        hold on
        scatter3(landmark_locations(1,:),landmark_locations(2,:),landmark_locations(3,:),'k')
        scatter3(meas_world(1,:), meas_world(2,:), meas_world(3,:), 'b*')
        hold off
        xlabel("X");
        ylabel("Y");
        zlabel("Z");
        %xlim([-(map_size + 5), (map_size + 5)])
        %ylim([-(map_size + 5), (map_size + 5)])
        axis square
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

%% Run simulation
for i=2:size(time_vec,2)
    % Generate measurement sets
    [meas, landmark_inFOV] = gen_noisy_meas (pos(:,i), quat(i,:), landmark_locations, sensor);
    

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
        view([0,90])
        hold on
        scatter3(landmark_locations(1,:),landmark_locations(2,:),landmark_locations(3,:),'k')
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
        
        map_name = strcat('/home/tuan/Projects/anarsh/visual_RB_PHD_SLAM/map/',indx_name,'.png');
        meas_name = strcat('/home/tuan/Projects/anarsh/visual_RB_PHD_SLAM/meas/',indx_name,'.png');
        %saveas (fig1,map_name);
        %saveas (fig2, meas_name);
        drawnow
    end
end



