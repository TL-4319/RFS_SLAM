% Simulation. The math is general for 3D but the simulation is limit to 2D
close all;
clear;
clc;

addpath("util/")
rng(69431);
dt = 0.2;
draw = false;

image_rate = 5;
imu_rate = 100;

image_dt = 1/image_rate;
imu_dt = 1/imu_rate;

%% Generate landmark map
map_size = 100;
num_landmark = 2000;
landmark_locations = vertcat((rand(1,num_landmark)-0.2) * 2 * map_size ,...
    (rand(1,num_landmark)-0.2) * 2 * map_size, (rand(1,num_landmark)-0.5) * 2 * map_size);
marker_size = ones(size(landmark_locations,2),1) * 10;

%% Sensor/Robot
% Sensor properties
sensor.HFOV = deg2rad(110);
sensor.VFOV = deg2rad(70);
sensor.max_range = 50;
sensor.min_range = 0.4;
sensor.P_d = 0.8;
sensor.clutter_rate = 2;
sensor.sigma = 0.1;

% Generate trajectory
waypoints = [0,0,0; ... % Initial position
             20, 10, 0;...
             20, 60, 0; ...
             40, 80, 0
             70,100,0];    % Final position

orientation_wp = quaternion([0,0,0; ...
                          30,10,0;...
                          90,0,0;...
                          70,0,0;...
                          50,0,0],...
                          "eulerd","ZYX","frame");

groundspeed = ones(1,size(waypoints,1)) * 1; groundspeed(1) = 0; %Initial zero velocity

[pos, quat, trans_vel, acc_body, acc_world, rot_vel, rot_vel_world, imu_time_vec] = generate_trajectory2(waypoints,...
    orientation_wp, groundspeed, imu_dt);

%% Generate IMU data
% No bias imu
IMU_no_bias = imuSensor('accel-gyro','SampleRate',imu_dt);
% Noise characteristic of IMU
IMU_no_bias.Accelerometer = accelparams( ...
    'MeasurementRange',19.62, ...            % m/s^2
    'NoiseDensity',0.0028);               % m/s^2 / Hz^(1/2)

IMU_no_bias.Gyroscope = gyroparams(...
    'MeasurementRange',4.363, ...   % rad/s
    'NoiseDensity', 0.00016);      % rad/s / Hz^(1/2)
[no_bias_accel, no_bias_gyro] = IMU_no_bias(acc_world', rot_vel', quat);

%% Generate img timestamps
has_img = zeros(size(imu_time_vec));
has_img(1) = 1;
img_time_vec = 0;
cur_time = 0;
last_time = 0;
for t = 2:size(imu_time_vec,2)
    cur_time = cur_time + imu_dt;
    delta = cur_time-last_time;
    if abs(delta - image_dt) <= 0.001
        img_time_vec = horzcat(img_time_vec, cur_time);
        has_img(t) = 1;
        last_time = cur_time;
    end
end


%% Prealocate truth data
truth.pos = pos;
truth.body_trans_vel = trans_vel;
truth.body_rot_vel = rot_vel;
truth.body_trans_accel = acc_body;
truth.quat = quat;
truth.imu_time_vec = imu_time_vec;
truth.landmark_locations = landmark_locations;
truth.landmark_in_FOV = cell(size(img_time_vec,2),1);
truth.time_vec = img_time_vec;
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

%% Run measurement simulation
k = 2;
for i=2:size(imu_time_vec,2)
    disp(imu_time_vec(i))
    
    % Only generate measurement at correct time
    if has_img(i) == 1
        % Generate measurement sets
        [meas, landmark_inFOV] = gen_noisy_3D_meas (pos(:,i), quat(i,:), landmark_locations, sensor);
        
    
        % Reproject meas into world frame for double checking
        meas_world = reproject_meas (pos(:,i), quat(i,:), meas);
        
        truth.landmark_in_FOV{k,1} = landmark_inFOV;
        truth.meas_table{k,1} = meas;
        k = k + 1;
    end


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
        
        map_name = strcat('/home/tuan/Projects/anarsh/visual_RB_PHD_SLAM/map/',indx_name,'.png');
        meas_name = strcat('/home/tuan/Projects/anarsh/visual_RB_PHD_SLAM/meas/',indx_name,'.png');
        %saveas (fig1,map_name);
        %saveas (fig2, meas_name);
        drawnow
    end
end



