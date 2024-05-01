close all;
clear;
clc;

%% 
rng(307);
draw = true;

%% Load truth and measurement data
addpath ('../util/')

load('../dataset/truth_3D_imu_0_100hz.mat');
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
est.world_vel = truth.world_vel;
est.compute_time = zeros(1,size(imu_time_vec,2));
est.grav_vec = [0; 0; 9.81];
est.gyro_bias = zeros(3,size(imu_time_vec,2));
est.acc_bias = zeros(3,size(imu_time_vec,2));

%% Generate IMU measurements
% IMU parameters
imu_param.gyro_NoiseDensity = 0.00028; % rad/s / Hz^(1/2)
imu_param.gyro_Bias = [-1, 2, 3] * 10^-2; % gyro constant bias term (rad)
imu_param.accel_NoiseDensity = 0.00018; % m/s^2 / Hz^(1/2)
imu_param.accel_Bias = [-5, 5, 3] * 10^-1; % accel constant bias term (m/s)
%imu_param.accel_Bias = [0, 0, 0] * 10^-2; % accel constant bias term (m/s)
imu_param.dt = imu_dt;

% Generate IMU meas
imu_meas = generate_imu_measurements(truth.world_accel, truth.world_rot_vel, truth.quat, imu_param);

%% SLAM configuration
% State vector consists of 
% Non linear components: pos, quat, vel
% Linear components: gyro_bias, accel_bias

% Trajectory config
filter_params.num_particle = 100;

% Motion covariance = [acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z]
filter_params.motion_sigma = [0.02; 0.02; 0.02; 0.02; 0.02; 0.02];

% Linear noise parameters
filter_params.cov_imu_noise = diag([0.02; 0.02; 0.02; 0.02; 0.02; 0.02].^2);
filter_params.cov_bias_noise = diag([5,5,5,5,5,5] * 10^-4);

% Initial values for linear sub-components
filter_params.init_gyro_bias = [0; 0; 0];
filter_params.init_acc_bias = [0; 0; 0];
filter_params.init_bias_P = diag([1, 1, 1, 1, 1, 1] * 10^-2);

% Map PHD config
filter_params.birthGM_intensity = 0.01;
filter_params.birthGM_cov = [0.02, 0, 0; 0, 0.02, 0; 0, 0, 0.02].^2;

% Sensor model
filter_params.map_Q = diag([0.01, 0.01, 0.01].^2);
filter_params.filter_sensor_noise = 0.1;
filter_params.R = diag([filter_params.filter_sensor_noise^2, ...
    filter_params.filter_sensor_noise^2, filter_params.filter_sensor_noise^2]);
%clutter_intensity = sensor.clutter_rate / (sensor.Range^2 * sensor.HFOV * 0.5) * 1e-4;
filter_params.clutter_intensity = 2 / (15^2 * 0.3 * 0.1 * pi^2);
filter_params.P_d = 0.8;

% PHD management parameters
filter_params.pruning_thres = 10^-5;
filter_params.merge_dist = 4;
filter_params.num_GM_cap = 5000;

est.filter_params = filter_params;
est.num_effective_part = zeros (1,size(imu_time_vec,2));

% Select which IMU to used
est.imu_used = "imu";
if est.imu_used == "perfect"
    est.imu_meas = imu_meas.perfect_imu;
elseif est.imu_used == "no_bias"
    est.imu_meas = imu_meas.no_bias_imu;
elseif est.imu_used == "imu"
    est.imu_meas = imu_meas.imu;
end


%% Initialize SLAM particles
cur_pos = est.pos(:,1);
cur_quat = est.quat(1,:);
cur_vel = est.world_vel(:,1);
meas = truth.meas_table{1,1};
meas_world_frame = reproject_meas(cur_pos, cur_quat, meas);
particles = init_phd_particles_marginalized (filter_params.num_particle, cur_pos, cur_vel, cur_quat,...
    filter_params.init_gyro_bias, filter_params.init_acc_bias, filter_params.init_bias_P,...
    meas_world_frame, filter_params.birthGM_cov, filter_params.birthGM_intensity);
img_ind = 1;

%% Run simulation
for i=2:size(imu_time_vec,2)
    %% Parse some truth data to reproject measurement into map space
    meas = truth.meas_table{img_ind,1};
    meas_reprojected = reproject_meas(truth.pos(:,i), truth.quat(i,:), meas);
    

    pause_ind = -1;
    if i == pause_ind + 1
        disp("Pause");
    end
    
    %% Propagate IMU to generate odometry
    tic
    %% Get inertial measurements
    cur_imu_gyro = est.imu_meas.gyro(:,i);
    cur_imu_acc = est.imu_meas.accel(:,i);
    for par_ind = 1:size(particles,2)
        cur_particle = particles(1,par_ind);
        %% MPF time update. Step 3 of Alg 2.4 in Blesser's thesis 
        %% Step 3.a Particle filter time update
        % Generate sampled measurement for accel and gyro for propagation
        acc_noise_sample = normrnd(0,1,3,1) .* filter_params.motion_sigma(1:3);
        sampled_acc = cur_imu_acc + acc_noise_sample;
        
        gyr_noise_sample = normrnd(0,1,3,1) .* filter_params.motion_sigma(4:6);
        sampled_gyr = cur_imu_gyro + gyr_noise_sample;
        
        % Save prev non_linear state
        prev_non_linear_state.pos = cur_particle.pos;
        prev_non_linear_state.vel = cur_particle.vel;
        prev_non_linear_state.quat = cur_particle.quat;

        % Propagate forward in time given measurements
        [cur_particle.pos, cur_particle.vel, cur_particle.quat,~, ~] = ...
        propagate_imu(cur_particle.pos, cur_particle.vel, cur_particle.quat, ...
        sampled_acc, sampled_gyr, cur_particle.gyro_bias, cur_particle.acc_bias, est.grav_vec, imu_dt);
        % [cur_particle.pos, cur_particle.vel, cur_particle.quat,~, ~] = ...
        % propagate_imu(cur_particle.pos, cur_particle.vel, cur_particle.quat, ...
        % sampled_acc, sampled_gyr, est.gyro_bias(:,i), est.acc_bias(:,i), est.grav_vec, imu_dt);
        
        
        % Save predicted non_linear state
        pred_non_linear_state.pos = cur_particle.pos;
        pred_non_linear_state.vel = cur_particle.vel;
        pred_non_linear_state.quat = cur_particle.quat;

        %% Step 3.b Kalman filter corrective measurement update 
        [bias_correct, P_correct] = MPF_corrective_KF_meas_update(prev_non_linear_state,...
            pred_non_linear_state, [cur_particle.gyro_bias;cur_particle.acc_bias], ...
            cur_particle.bias_cov, filter_params.cov_imu_noise, sampled_acc, ...
            sampled_gyr, est.grav_vec, imu_dt);

        cur_particle.gyro_bias = bias_correct(1:3);
        cur_particle.acc_bias = bias_correct(4:6);
        cur_particle.bias_cov = P_correct;

        %% Step 3.c Kalman filter time update
        [bias_pred, P_pred] = MPF_KF_time_update ([cur_particle.gyro_bias;cur_particle.acc_bias],...
            cur_particle.bias_cov, filter_params.cov_bias_noise);
        
        cur_particle.gyro_bias = bias_pred(1:3);
        cur_particle.acc_bias = bias_pred(4:6);
        cur_particle.bias_cov = P_pred;

        particles(1,par_ind) = cur_particle;
    end
    
    if truth.has_img(i) == 1
        %% MPF measurement update
        % Fuse camera measurement when it is available
        Parlikeli = zeros (1,filter_params.num_particle);
        for par_ind = 1:size(particles,2)
            cur_particle = particles(1,par_ind);
    
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

            num_GM_in = size(GM_inten_in,2);
    
            %% Only  perform update steps if there are GM components in the FOV
            if num_GM_in > 0
                %% Porpagate dynamics of landmark. Landmark has no motion so mu stay
                % constant. 
                for kk = 1:num_GM_in
                    GM_cov_in(:,:,kk) = GM_cov_in_prev(:,:,kk) + filter_params.map_Q;
                end
                
                %% Pre compute inner update terms
                [pred_z, K, S, P, Sinv] = compute_update_terms (cur_particle,...
                    GM_mu_in_prev, GM_cov_in_prev, filter_params.R);

                %% Compute particle likelihood - single cluster 
                likelipz = zeros (1,size(meas,2));
                for jj = 1:size(meas,2)
                    likelipf = zeros (1,num_GM_in);
                    for kk = 1:num_GM_in
                        zdiff = meas(:, jj) - pred_z(:,kk);
                        %%%%%
                        likelipf (1,kk) = 1 / sqrt(det(S(:,:,kk)) * (2*pi) ^size(filter_params.R,1))*...
                        exp(-0.5*(zdiff)' * Sinv(:,:,kk) * zdiff ) * ...
                        GM_inten_in_prev(kk);
                    end
                    likelipz(1,jj) = filter_params.clutter_intensity + sum (likelipf * filter_params.P_d,2);
                end
                 Parlikeli(1,par_ind) = exp(sum(GM_inten_in_prev,2)) * (prod(likelipz,2) + 1e-99) * cur_particle.w;
    
                %% Static map update
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
    
        %% EAP for state estimation or weighted sum
        est_pos = [0;0;0];
        est_euler = [0;0;0]; % Uses euler to be able to perform weighted sum on orientation. Sequence ZYX
        est_vel = [0;0;0];
        est_gyro_bias = [0;0;0];
        est_acc_bias = [0;0;0];
        particle_likeli = zeros(1, size(particles,2)); % Use map estimate from max likeli
        for par_ind = 1:size(particles,2)
            est_pos = est_pos + particles(1,par_ind).w * particles(1,par_ind).pos;
            est_vel = est_vel + particles(1,par_ind).w * particles(1,par_ind).vel;
            est_gyro_bias = est_gyro_bias + particles(1,par_ind).w * particles(1,par_ind).gyro_bias;
            est_acc_bias = est_acc_bias + particles(1,par_ind).w * particles(1,par_ind).acc_bias;

            cur_ind_euler = quat2eul(particles(1,par_ind).quat); %ZYX
            est_euler = est_euler + cur_ind_euler' * particles(1,par_ind).w;

            particle_likeli(1,par_ind) = particles(1,par_ind).w;
        end
        
        est.pos(:,i) = est_pos;
        est.vel(:,i) = est_vel;
        est.gyro_bias(:,i) = est_gyro_bias;
        est.acc_bias(:,i) = est_acc_bias;
        temp = eul2quat(est_euler');
        est.quat(i,:) = quaternion(temp);
        
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
            if min(dist) >= 0.3 % If the measurement is not close to any existing landmark/target
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
    
        %% Resample particles
        wei_ud = Parlikeli / sum(Parlikeli,2); %normalize weight
        est.num_effective_part(:,i) = 1/ sum(wei_ud.^2,2);
        if est.num_effective_part(:,i) < 0.5 * filter_params.num_particle
            disp('Resample triggered');
            
            resample_ind = low_variance_resample(wei_ud, filter_params.num_particle);
            for par_ind = 1:filter_params.num_particle
                particles(1,par_ind).pos = particles(1,resample_ind(1,par_ind)).pos;
                particles(1,par_ind).quat = particles(1,resample_ind(1,par_ind)).quat;
                particles(1,par_ind).gm_mu = particles(1,resample_ind(1,par_ind)).gm_mu;
                particles(1,par_ind).gm_inten = particles(1,resample_ind(1,par_ind)).gm_inten;
                particles(1,par_ind).gm_cov = particles(1,resample_ind(1,par_ind)).gm_cov;
                particles(1,par_ind).gyro_bias = particles(1,resample_ind(1,par_ind)).gyro_bias;
                particles(1,par_ind).acc_bias = particles(1,resample_ind(1,par_ind)).acc_bias;
                particles(1,par_ind).bias_cov = particles(1,resample_ind(1,par_ind)).bias_cov;
                particles(1,par_ind).w = 1/filter_params.num_particle;
            end
        end
        est.compute_time(1,i) = toc;
        %% Plotting
    
         if draw
            % Plotting
            figure(1)
            draw_trajectory(truth.pos(:,i), truth.quat(i,:), truth.pos(:,1:i), 1, 5, 2,'k',false);
            draw_trajectory(est.pos(:,i), est.quat(i,:), est.pos(:,1:i), 1, 5, 2,'g',true);
            hold on
            set(gca, 'Zdir', 'reverse')
            set(gca, 'Ydir', 'reverse')
            grid on
            scatter3(truth.cumulative_landmark_in_FOV{end,1}(1,:),...
                truth.cumulative_landmark_in_FOV{end,1}(2,:),...
                truth.cumulative_landmark_in_FOV{end,1}(3,:),...
                ones(size(truth.cumulative_landmark_in_FOV{end,1},2),1) * 10,'k')
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
            zlim([-4 2])
            title_str = sprintf("Expected num of landmark = %d. t = %f", exp_num_landmark,img_time_vec(img_ind));
            title(title_str)
            %exportgraphics(fig1, "map.gif", Append=true);
    
            % figure(2)
            % draw_trajectory(truth.pos(:,i), truth.quat(i,:), [0;0;0], 1, 10, 2,'k',false);
            % hold on
            % draw_phd(max_likeli_gm_mu, max_likeli_gm_cov, max_likeli_gm_inten,[-30 150], truth.landmark_locations,"Test")
            % %exportgraphics(fig2, "phd5.gif", Append=true);
            % drawnow;
         end
         img_ind = img_ind + 1;
         dbg_str = sprintf("timestep %f, num_landmark %d, num effective sample %d",img_time_vec(img_ind),exp_num_landmark, est.num_effective_part(:,i));
         disp(dbg_str);
    
    else 
        %% If no camera measurement yet
        %% EAP for state estimation or weighted sum
        est_pos = [0;0;0];
        est_euler = [0;0;0]; % Uses euler to be able to perform weighted sum on orientation. Sequence ZYX
        est_vel = [0;0;0];
        est_gyro_bias = [0;0;0];
        est_acc_bias = [0;0;0];
        particle_likeli = zeros(1, size(particles,2)); % Use map estimate from max likeli
        for par_ind = 1:size(particles,2)
            est_pos = est_pos + particles(1,par_ind).w * particles(1,par_ind).pos;
            est_vel = est_vel + particles(1,par_ind).w * particles(1,par_ind).vel;
            est_gyro_bias = est_gyro_bias + particles(1,par_ind).w * particles(1,par_ind).gyro_bias;
            est_acc_bias = est_acc_bias + particles(1,par_ind).w * particles(1,par_ind).acc_bias;

            cur_ind_euler = quat2eul(particles(1,par_ind).quat); %ZYX
            est_euler = est_euler + cur_ind_euler' * particles(1,par_ind).w;

            particle_likeli(1,par_ind) = particles(1,par_ind).w;
        end
        
        est.pos(:,i) = est_pos;
        est.vel(:,i) = est_vel;
        est.gyro_bias(:,i) = est_gyro_bias;
        est.acc_bias(:,i) = est_acc_bias;
        temp = eul2quat(est_euler');
        est.quat(i,:) = quaternion(temp);

        %% MAP Landmark estimation
        [max_likeli, max_w_particle_ind] = max(particle_likeli);
        max_likeli_gm_mu = particles(1,max_w_particle_ind).gm_mu;
        max_likeli_gm_inten = particles(1,max_w_particle_ind).gm_inten;
        max_likeli_gm_cov = particles(1,max_w_particle_ind).gm_cov;
        % Find expected number of landmark
        exp_num_landmark = round(sum (max_likeli_gm_inten));
        %ID_map = find(max_likeli_gm_inten > landmark_threshold);
        [~,ID_map] = maxk (max_likeli_gm_inten, exp_num_landmark);
        map_est = max_likeli_gm_mu(:,ID_map);
        est.map_est {i,1} = map_est;

        est.compute_time(1,i) = toc;
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