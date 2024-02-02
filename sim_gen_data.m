% Simulation. The math is general for 3D but the simulation is limit to 2D
close all;
clear;
clc;

rng(676);
dt = 0.2;
time_vec = 0:dt:100;
draw = true;
%% Generate landmark map
map_size = 70;
num_landmark = 100;
landmark_locations = (rand(num_landmark, 3) - 0.2) * 2 * map_size;
landmark_locations(:,3) = zeros(1,size(landmark_locations,1));
landmark_locations = landmark_locations';

%% Sensor/Robot
% Sensor properties
sensor.HFOV = deg2rad(100);
sensor.Range = 30;
sensor.P_d = 0.8;
sensor.clutter_rate = 2;
sensor.sigma = 0.1;

% Initial pose
pos = [0;0;0];
quat = quaternion(1, 0, 0, 0);
traj_hist = [];

body_vel = [1;0;0];
turn_command_delta = (rand(1, size(time_vec,2)) - 0.48);
turn_command_scale = rand(1, size(time_vec,2)) * 0.01;
body_rot_rate = zeros(3,size(time_vec,2));
body_rot_rate(3,:) = cumsum(turn_command_delta.*turn_command_scale);

%% Prealocate truth data
truth.pos = zeros(3,size(time_vec,2));
truth.pos(:,1) = pos;
truth.quat = quaternion (zeros(size(time_vec,2),4));
truth.quat(1,:) = quat;
truth.time_vec = time_vec;
truth.landmark_locations = landmark_locations;

truth.sensor = sensor;

meas_table = cell(size(time_vec,2),1);

if draw
    fig1 = figure(1);
    title ("Sim world")
    movegui (fig1, [0,0])
    
    fig2 = figure(2);
    title("Sensor frame")
    movegui (fig2, [570,0])
end
% Draw first frame

% Generate measurement sets
meas = gen_meas (pos, quat, landmark_locations, sensor);

% Reproject meas into world frame for double checking
meas_world = reproject_meas (pos, quat, meas);

meas_table{1,1} = meas;

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
    
        figure(2)
        scatter3(meas(1,:), meas(2,:), meas(3,:), 'b*')
        xlabel("X");
        ylabel("Y");
        set(gca, 'Zdir', 'reverse')
        set(gca, 'Ydir', 'reverse')
        grid on
        view([-90,90])
        xlim([-sensor.Range, sensor.Range])
        ylim([-sensor.Range, sensor.Range])
        hold on
        plot3 ([0,10],[0,0], [0,0],'r',LineWidth=2)
        plot3 ([0,0],[0,10], [0,0],'g',LineWidth=2)
        hold off
        axis square
        
        map_name = strcat('/home/tuan/Projects/anarsh/visual_RB_PHD_SLAM/map/','001','.png');
        meas_name = strcat('/home/tuan/Projects/anarsh/visual_RB_PHD_SLAM/meas/','001','.png');
        saveas (fig1,map_name);
        %saveas (fig2, meas_name);
        drawnow
    end

%% Run simulation
for i=2:size(time_vec,2)
    % Update robot pose
    traj_hist = horzcat (traj_hist,pos);
    [pos, quat] = propagate_state(pos, quat, body_vel, body_rot_rate(:,i), dt);

    % Generate measurement sets
    meas = gen_meas (pos, quat, landmark_locations, sensor);

    % Reproject meas into world frame for double checking
    meas_world = reproject_meas (pos, quat, meas);
    
    truth.pos(:,i) = pos;
    truth.quat(i,:) = quat;
    meas_table{i,1} = meas;


    if draw
        % Plotting
        figure(1)
        draw_trajectory(pos, quat, traj_hist, 1, 10, 2,'k',false);
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
    
        figure(2)
        scatter3(meas(1,:), meas(2,:), meas(3,:), 'b*')
        xlabel("X");
        ylabel("Y");
        set(gca, 'Zdir', 'reverse')
        set(gca, 'Ydir', 'reverse')
        grid on
        view([-90,90])
        xlim([-sensor.Range, sensor.Range])
        ylim([-sensor.Range, sensor.Range])
        hold on
        plot3 ([0,10],[0,0], [0,0],'r',LineWidth=2)
        plot3 ([0,0],[0,10], [0,0],'g',LineWidth=2)
        hold off
        axis square
        
        map_name = strcat('/home/tuan/Projects/anarsh/visual_RB_PHD_SLAM/map/',indx_name,'.png');
        meas_name = strcat('/home/tuan/Projects/anarsh/visual_RB_PHD_SLAM/meas/',indx_name,'.png');
        saveas (fig1,map_name);
        %saveas (fig2, meas_name);
        drawnow
    end
end



