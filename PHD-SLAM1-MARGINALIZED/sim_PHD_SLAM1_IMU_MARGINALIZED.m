% Simulation. The math is general for 3D but the simulation is limit to 2D
close all;
clear;
clc;

%% 
rng(307);
draw = false;

%% Load truth and measurement data
addpath ('../util/')

load('../dataset/truth_3D_imu_0_200hz.mat');
marker_size = ones(size(truth.landmark_locations,2),1) * 10;

%% Time vector
imu_time_vec = truth.imu_time_vec;
imu_dt = imu_time_vec(2) - imu_time_vec(1);
img_time_vec = truth.img_time_vec;
img_dt = img_time_vec(2) - img_time_vec(1);

%% Drawing stuffs
if draw
    fig1 = figure(1);
    title ("Sim world")
    fig1.Position = [1,1,800,800];
end


%% Prealocate estimated traj
est.pos = truth.pos;
est.quat = truth.quat;
est.compute_time = zeros(1,size(imu_time_vec,2));

%% Generate IMU measurements
% IMU parameters
imu_param.accel_NoiseDensity = 0.0028;
imu_param.accel_RandomWalk = 0.000086;
imu_param.gyro_NoiseDensity = 0.00016;
imu_param.gyro_RandomWalk = 0.0000022;
imu_param.dt = imu_dt;

% Generate IMU meas
imu_meas = generate_imu_measurements(truth.world_accel, truth.world_rot_vel, truth.quat, imu_param);

%% SLAM configuration
% Trajectory config
filter_params.num_particle = 1;
% Motion covariance = [acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z]
filter_params.motion_sigma = [0.3; 0.3; 0.3; 0.01; 0.01; 0.01];

% Map PHD config
filter_params.birthGM_intensity = 0.1;
filter_params.birthGM_cov = [0.2, 0, 0; 0, 0.2, 0; 0, 0, 0.2];

% Sensor model
filter_params.map_Q = diag([0.1, 0.1, 0.1]);
filter_params.filter_sensor_noise = 0.1;
filter_params.R = diag([filter_params.filter_sensor_noise^2, ...
    filter_params.filter_sensor_noise^2, filter_params.filter_sensor_noise^2]);
%clutter_intensity = sensor.clutter_rate / (sensor.Range^2 * sensor.HFOV * 0.5) * 1e-4;
filter_params.clutter_intensity = 3 / (20^2 * 0.3 * pi);
filter_params.P_d = 0.8;

% PHD management parameters
filter_params.pruning_thres = 10^-5;
filter_params.merge_dist = 0.3;
filter_params.num_GM_cap = 5000;

est.filter_params = filter_params;

%% Initialize SLAM particles
cur_pos = est.pos(:,1);
cur_quat = est.quat(1,:);
meas = truth.meas_table{1,1};
meas_world_frame = reproject_meas(cur_pos, cur_quat, meas);
particles = init_phd_particles (filter_params.num_particle, cur_pos, cur_quat, ...
    meas_world_frame, filter_params.birthGM_cov, filter_params.birthGM_intensity);

%% Run simulation
for i=2:size(imu_time_vec,2)
    %% Parse some truth data
    meas = truth.meas_table{i,1};
    meas_reprojected = reproject_meas(truth.pos(:,i), truth.quat(i,:), meas);
    

    pause_ind = -1;
    if i == pause_ind + 1
        disp("Pause");
    end
    
    %% Propagate IMU to generate odometry
    tic

    % Fuse measurement when it is available
    if truth.has_img(i) == 1
        Parlikeli = zeros (1,filter_params.num_particle);
        for par_ind = 1:size(particles,2)
            cur_particle = particles(1,par_ind);
            % %% Trajectory prediction
            body_vel_sample = randn(3,1) .* filter_params.motion_sigma(1:3);
            body_rot_vel_sample = randn(3,1) .* filter_params.motion_sigma(4:6); 
            
            % % Particle odometry sample
            body_vel_sample = odom.body_trans_vel(:,i) + body_vel_sample;
            body_rot_vel_sample = odom.body_rot_vel(:,i) + body_rot_vel_sample;
    
            [cur_particle.pos, cur_particle.quat] = propagate_state (cur_particle.pos, ...
                cur_particle.quat,body_vel_sample, body_rot_vel_sample, dt);
    
            % Give particle truth pose for map debug
            cur_particle.pos = truth.pos(:,i);
            cur_particle.quat = truth.quat(i,:);
    
            particles(1,par_ind).pos = cur_particle.pos;
            particles(1,par_ind).quat = cur_particle.quat;
        
    
            % figure(2)
            % draw_trajectory(truth.pos(:,i), truth.quat(i,:), [0;0;0], 1, 10, 2,'k',false);
            % str_name = sprintf("All GM at %d", i);
            % hold on
            % draw_phd(cur_particle.gm_mu, cur_particle.gm_cov, cur_particle.gm_inten,[-30 150], truth.landmark_locations,str_name)
             
    
            %% Check if GM is in FOV
            num_GM = size(cur_particle.gm_mu,2);
            [~, GM_in_FOV] = check_in_FOV_3D (cur_particle.gm_mu, ...
                    cur_particle.pos, cur_particle.quat, truth.sensor_params);
    
            %% Extract GM components not in FOV. No changes are made to them
            GM_out_FOV = ~GM_in_FOV;
            GM_mu_out = cur_particle.gm_mu(:,GM_out_FOV);
            GM_cov_out = cur_particle.gm_cov (:,:,GM_out_FOV);
            GM_inten_out = cur_particle.gm_inten(GM_out_FOV);
    
            %% Extract GM components in FOV. These are used during update
            % Predict 
            GM_mu_in = cur_particle.gm_mu(:,GM_in_FOV);
            GM_cov_in = cur_particle.gm_cov (:,:,GM_in_FOV);
            GM_inten_in = cur_particle.gm_inten(GM_in_FOV);
    
            % Copy a set of previous GM k|k-1 for use in update step
            GM_mu_in_prev = GM_mu_in;
            GM_cov_in_prev = GM_cov_in;
            GM_inten_in_prev = GM_inten_in;
            
            %
            % figure(3)
            % draw_trajectory(truth.pos(:,i), truth.quat(i,:), [0;0;0], 1, 10, 2,'k',false);
            % str_name = sprintf("GM out FOV at %d", i);
            % hold on
            % draw_phd(GM_mu_out, GM_cov_out, GM_inten_out,[-30 150], truth.landmark_locations,str_name)
            % 
            % figure(4)
            % draw_trajectory(truth.pos(:,i), truth.quat(i,:), [0;0;0], 1, 10, 2,'k',false);
            % str_name = sprintf("GM in FOV at %d", i);
            % hold on
            % draw_phd(GM_mu_in, GM_cov_in, GM_inten_in,[-30 150], truth.landmark_locations,str_name)
            
    
            num_GM_in = size(GM_inten_in,2);
    
            %% Only  perform update steps if there are GM components in the FOV
            if num_GM_in > 0
                % Porpagate dynamics of landmark. Landmark has no motion so mu stay
                % constant. 
            
                for kk = 1:num_GM_in
                    GM_cov_in(:,:,kk) = GM_cov_in_prev(:,:,kk) + filter_params.map_Q;
                end
    
    
                % Compute particle likelihood - NEED MORE RESEARCH
                likelipz = zeros (1,size(meas,2));
                for jj = 1:size(meas,2)
                    likelipf = zeros (1,num_GM_in);
                    for kk = 1:num_GM_in
                        pred_z = gen_pred_meas(cur_particle.pos, cur_particle.quat, GM_mu_in_prev(:,kk));
                        zdiff = meas(:, jj) - pred_z;
                        %%%%%
                        likelipf (1,kk) = 1/sqrt(det(2*pi*filter_params.R))*...
                            exp(-0.5*(zdiff)'*inv(filter_params.R)*zdiff ) * ...
                            GM_inten_in_prev(kk);
                        %likelipf (1, kk) = mvnpdf(meas(:, jj), pred_z, R) * clutter_intensity;
                    end
                    likelipz(1,jj) = filter_params.clutter_intensity + sum (likelipf,2);
                end
                Parlikeli(1,par_ind) = prod(likelipz,2) + 1e-99;
    
    
                % Pre compute inner update terms
                [pred_z, K, S, P] = compute_update_terms (cur_particle,...
                    GM_mu_in_prev, GM_cov_in_prev, filter_params.R);
    
                % Update missed detection terms
                GM_inten_in = (1 - filter_params.P_d) * GM_inten_in_prev;
    
                %Update PHD component of detected 
                l = 0;
                for zz = 1:size(meas,2)
                    l = l + 1;
                    tau = zeros(1,num_GM_in);
                    for jj = 1:num_GM_in
                        tau(1,jj) = filter_params.P_d * ...
                            GM_inten_in_prev(jj) * ...
                            mvnpdf(meas(:,zz),pred_z(:,jj),S(:,:,jj));
                        mu = GM_mu_in_prev(:,jj) + K(:,:,jj)* (meas(:,zz) - pred_z(:,jj));
                        GM_mu_in = horzcat(GM_mu_in, mu);
                        GM_cov_in = cat(3,GM_cov_in, P(:,:,jj));
                    end
                        tau_sum = sum(tau);
                    for jj = 1:num_GM_in
                        nu = tau(jj) / (filter_params.clutter_intensity + tau_sum);
                        GM_inten_in = horzcat(GM_inten_in, nu);
                    end
                end
                %% Clean up GM components
                % Prune
                [GM_mu_in, GM_cov_in, GM_inten_in] = cleanup_PHD (GM_mu_in,...
                    GM_cov_in, GM_inten_in, filter_params.pruning_thres, ...
                    filter_params.merge_dist, filter_params.num_GM_cap);
        
                %% Add back components not in FOV
                particles(1,par_ind).gm_mu = cat(2,GM_mu_in, GM_mu_out);
                particles(1,par_ind).gm_inten = cat (2, GM_inten_in, GM_inten_out);
                particles(1,par_ind).gm_cov = cat(3,GM_cov_in, GM_cov_out);
            %% If there are no map GM in FOV of particle. Force that particle to be eliminated
            else
                particles(1,par_ind).w = 1e-99;
            end
    
        end
        %% State estimation (max likelihood)
        % Trajectory estimation
        [max_likeli, max_w_particle_ind] = max(Parlikeli);
    
        % MAP trajectory estimation
        est.pos(:,i) = particles(1,max_w_particle_ind).pos;
        est.quat(i,:) = particles(1,max_w_particle_ind).quat;
        
        % MAP Landmark estimation
        max_likeli_gm_mu = particles(1,max_w_particle_ind).gm_mu;
        max_likeli_gm_inten = particles(1,max_w_particle_ind).gm_inten;
        max_likeli_gm_cov = particles(1,max_w_particle_ind).gm_cov;
        % Find expected number of landmark
        exp_num_landmark = round(sum (max_likeli_gm_inten));
        %ID_map = find(max_likeli_gm_inten > landmark_threshold);
        [~,ID_map] = maxk (max_likeli_gm_inten, exp_num_landmark);
        map_est = max_likeli_gm_mu(:,ID_map);
        est.map_est {i,1} = map_est;
    
        %% Adaptive birth PHD (from Lin Gao paper)
        new_birth_mu = []; new_birth_inten = []; new_birth_cov = [];
        new_meas_world_frame = reproject_meas(est.pos(:,i), est.quat(i,:), meas);
        for zz = 1:size(meas,2)
            cur_GM_mu = particles(1,max_w_particle_ind).gm_mu;
            matrix_dist = repmat(new_meas_world_frame(:,zz),1, size(cur_GM_mu,2)) - cur_GM_mu;
            dist = vecnorm(matrix_dist);
            if min(dist) >= 5 % If the measurement is not close to any existing landmark/target
                new_birth_inten = horzcat (new_birth_inten, filter_params.birthGM_intensity);
                new_birth_mu = cat (2,new_birth_mu, new_meas_world_frame(:,zz));
                new_birth_cov = cat (3, new_birth_cov, filter_params.birthGM_cov);
            end
        end
        for par_ind = 1:size(particles,2)
            particles(1,par_ind).gm_cov = cat(3,particles(1,par_ind).gm_cov, new_birth_cov);
            particles(1,par_ind).gm_inten = horzcat(particles(1,par_ind).gm_inten, new_birth_inten);
            particles(1,par_ind).gm_mu = horzcat(particles(1,par_ind).gm_mu, new_birth_mu);
        end
    
        %% Resample particles (from Lin Gao)
        wei_ud = Parlikeli / sum(Parlikeli,2);
        resample_ind = particle_resample(wei_ud, filter_params.num_particle);
        for par_ind = 1:filter_params.num_particle
            particles(1,par_ind).pos = particles(1,resample_ind(1,par_ind)).pos;
            particles(1,par_ind).quat = particles(1,resample_ind(1,par_ind)).quat;
            particles(1,par_ind).w = particles(1,resample_ind(1,par_ind)).w;
            particles(1,par_ind).gm_mu = particles(1,resample_ind(1,par_ind)).gm_mu;
            particles(1,par_ind).gm_inten = particles(1,resample_ind(1,par_ind)).gm_inten;
            particles(1,par_ind).gm_cov = particles(1,resample_ind(1,par_ind)).gm_cov;
        end
        est.compute_time(1,i) = toc;
        %% Plotting
    
         if draw
            % Plotting
            figure(1)
            draw_trajectory(truth.pos(:,i), truth.quat(i,:), truth.pos(:,1:i), 1, 10, 2,'k',false);
            draw_trajectory(est.pos(:,i), est.quat(i,:), est.pos(:,1:i), 1, 10, 2,'g',true);
            hold on
            set(gca, 'Zdir', 'reverse')
            set(gca, 'Ydir', 'reverse')
            grid on
            scatter3(truth.landmark_locations(1,:),truth.landmark_locations(2,:),truth.landmark_locations(3,:),marker_size,'k')
            scatter3(meas_reprojected(1,:), meas_reprojected(2,:), meas_reprojected(3,:), 'b*')
            scatter3(map_est(1,:), map_est(2,:), map_est(3,:),'r+')
            %for j=1:size(particles,2)
            %    scatter3(particles(j).pos(1), particles(j).pos(2), particles(j).pos(3),'r.');
            %end
            hold off
            %draw_particle_pos(particles,1)
            xlabel("X");
            ylabel("Y");
            zlabel("Z");
            %xlim([-(map_size + 5), (map_size + 5)])
            %ylim([-(map_size + 5), (map_size + 5)])
            % xlim ([min(truth.landmark_locations(1,:)), max(truth.landmark_locations(1,:))])
            % ylim([min(truth.landmark_locations(2,:)), max(truth.landmark_locations(2,:))])
            axis equal
            title_str = sprintf("Expected num of landmark = %d. is = %d", exp_num_landmark,i);
            title(title_str)
            %exportgraphics(fig1, "map.gif", Append=true);
    
            % figure(2)
            % draw_trajectory(truth.pos(:,i), truth.quat(i,:), [0;0;0], 1, 10, 2,'k',false);
            % hold on
            % draw_phd(max_likeli_gm_mu, max_likeli_gm_cov, max_likeli_gm_inten,[-30 150], truth.landmark_locations,"Test")
            % %exportgraphics(fig2, "phd5.gif", Append=true);
            % drawnow;
         end
         dbg_str = sprintf("timestep %f, num_landmark %d",time_vec(i),exp_num_landmark);
         disp(dbg_str);
    end
end
simulation.est = est;
simulation.truth = truth;
simulation.odom = odom;

figure(1)
draw_trajectory(truth.pos(:,i), truth.quat(i,:), truth.pos(:,1:i), 1, 10, 2,'k',false);
draw_trajectory(est.pos(:,i), est.quat(i,:), est.pos(:,1:i), 1, 10, 2,'g',true);
hold on
set(gca, 'Zdir', 'reverse')
set(gca, 'Ydir', 'reverse')
grid on
%view([0,90])

scatter3(truth.landmark_locations(1,:),truth.landmark_locations(2,:),truth.landmark_locations(3,:),'k')
scatter3(meas_reprojected(1,:), meas_reprojected(2,:), meas_reprojected(3,:), 'b*')
scatter3(map_est(1,:), map_est(2,:), map_est(3,:),'r+')
%for j=1:size(particles,2)
%    scatter3(particles(j).pos(1), particles(j).pos(2), particles(j).pos(3),'r.');
%end
hold off
%draw_particle_pos(particles,1)
xlabel("X");
ylabel("Y");
zlabel("Z");
%xlim([-(map_size + 5), (map_size + 5)])
%ylim([-(map_size + 5), (map_size + 5)])
xlim ([min(truth.landmark_locations(1,:)), max(truth.landmark_locations(1,:))])
ylim([min(truth.landmark_locations(2,:)), max(truth.landmark_locations(2,:))])
axis square
title_str = sprintf("Expected num of landmark = %d. is = %d", exp_num_landmark,i);
title(title_str)