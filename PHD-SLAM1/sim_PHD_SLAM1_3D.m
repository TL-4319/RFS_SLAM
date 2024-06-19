% Simulation. The math is general for 3D but the simulation is limit to 2D
close all;
clear;
clc;

%% 
rng(689);
draw = false;

%% Load truth and measurement data
addpath ('../util/')

load('../dataset/card_test.mat');
%load('../dataset/truth_3D_15hz_dense.mat');
%load('../dataset/meas_table_2.mat');
%truth_hist = truth.pos(:,1);
%truth.meas_table = meas_table;
%truth.sensor_params = truth.sensor;
%truth.sensor_params.max_range = truth.sensor_params.Range;
%truth.sensor_params.min_range = 0;
marker_size = ones(size(truth.landmark_locations,2),1) * 10;

%% Time vector
time_vec = truth.time_vec;
dt = time_vec(2) - time_vec(1);

%% Drawing stuffs
if draw
    fig1 = figure(1);
    title ("Sim world")
    fig1.Position = [1,1,2000,2000];
    
    % fig2 = figure(2);
    % title("Sensor frame")
    % fig2.Position = [611,1,600,600];
end


%% Prealocate estimated traj
est.pos = truth.pos;
est.quat = truth.quat;
est.compute_time = zeros(1,size(time_vec,2));

%% Odometry parameters
odom.sigma_trans = [0.1; 0.1; 0.1];
odom.sigma_rot = [0.03; 0.03; 0.03];
% odom.body_trans_vel = truth.body_trans_vel + ...
%     randn(3,size(truth.body_trans_vel,2)) .* repmat(odom.sigma_trans,1,size(truth.body_trans_vel,2));
% odom.body_rot_vel = truth.body_rot_vel + ...
%     randn(3,size(truth.body_rot_vel,2)) .* repmat(odom.sigma_rot,1,size(truth.body_rot_vel,2));
odom.body_trans_vel = truth.body_trans_vel + normrnd(0,odom.sigma_trans(1),3,size(truth.body_trans_vel,2));
odom.body_rot_vel = truth.body_rot_vel + normrnd(0,odom.sigma_rot(1),3,size(truth.body_rot_vel,2));

%% SLAM configuration
% Trajectory config
filter_params.num_particle = 1;
% Motion covariance = [cov_x, cov_y, cov_z, cov_phi, cov_theta, cov_psi]
filter_params.motion_sigma = [0.1; 0.1; 0.1; 0.03; 0.03; 0.03];

% Map PHD config
filter_params.birthGM_intensity = 0.1;
filter_params.birthGM_cov = [0.1, 0, 0; 0, 0.1, 0; 0, 0, 0.1];

% Sensor model
filter_params.map_Q = diag([0.01, 0.01, 0.01].^2);
filter_params.filter_sensor_noise = 0.1;
filter_params.R = diag([filter_params.filter_sensor_noise^2, ...
    filter_params.filter_sensor_noise^2, filter_params.filter_sensor_noise^2]);
filter_params.clutter_intensity = 2 / (15^2 * pi);
%filter_params.clutter_intensity = 2 / (15^2 * 0.2 * pi);
filter_params.P_d = 0.9;

% PHD management parameters
filter_params.pruning_thres = 10^-6;
filter_params.merge_dist = 10;
filter_params.num_GM_cap = 200;

est.filter_params = filter_params;
est.num_effective_part = zeros (1,size(time_vec,2));

%% Initialize SLAM particles
cur_pos = est.pos(:,1);
cur_quat = est.quat(1,:);
meas = truth.meas_table{1,1};
meas_world_frame = reproject_meas(cur_pos, cur_quat, meas);
particles = init_phd_particles (filter_params.num_particle, cur_pos, cur_quat, ...
    meas_world_frame, filter_params.birthGM_cov, filter_params.birthGM_intensity);

%% dummy var for plotting
num_lm_truth = zeros(1,size(time_vec,2));
exp_lm = zeros(1,size(time_vec,2));

%% Run simulation
for i=2:size(time_vec,2)
    % map_png_name = sprintf ("/home/tuan/Projects/anarsh/visual_RB_PHD_SLAM/map/%s.png",pad(sprintf('%d',i), 3, 'left','0'));
    % figure(1)
    % imshow(map_png_name);

    %% Parse some truth data
    meas = truth.meas_table{i,1};

    meas_reprojected = reproject_meas(truth.pos(:,i), truth.quat(i,:), meas);
    

    pause_ind = -1;
    if i == pause_ind + 1
        disp("Pause");
    end

    %% SLAM runs here
    tic
    Parlikeli = zeros (1,filter_params.num_particle);
    for par_ind = 1:size(particles,2)
        cur_particle = particles(1,par_ind);
        % %% Trajectory prediction
        % body_vel_sample = randn(3,1) .* filter_params.motion_sigma(1:3);
        % body_rot_vel_sample = randn(3,1) .* filter_params.motion_sigma(4:6);
        body_vel_sample = normrnd(0,filter_params.motion_sigma(1),3,1);
        body_rot_vel_sample = normrnd(0,filter_params.motion_sigma(4),3,1);
        % 
        % % Constraint vel to 2D
        % body_vel_sample(2:3,:) = 0;
        % body_rot_vel_sample(1:2,:) = zeros (2,1);
        % 
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

        num_GM_in = size(GM_inten_in,2);

        %% Only  perform update steps if there are GM components in the FOV
        if num_GM_in > 0
            % Porpagate dynamics of landmark. Landmark has no motion so mu stay
            % constant. 
        
            % for kk = 1:num_GM_in
            %     GM_cov_in(:,:,kk) = GM_cov_in_prev(:,:,kk) + filter_params.map_Q;
            % end


            %% Pre compute inner update terms
            [pred_z, K, S, P, Sinv] = compute_update_terms (cur_particle,...
                GM_mu_in_prev, GM_cov_in_prev, filter_params.R);

            %% Compute particle likelihood - single cluster
            likelipz = zeros (1,size(meas,2));
            for jj = 1:size(meas,2)
                likelipf = zeros (1,num_GM_in);
                for kk = 1:num_GM_in
                    zdiff = meas(:, jj) - pred_z(:,kk);
                    if GM_inten_in_prev(kk) < 0.8
                        continue
                    end
                    % likelipf (1,kk) = 1 / sqrt(det(S(:,:,kk)) * (2*pi) ^size(filter_params.R,1))*...
                    %     exp(-0.5*(zdiff)' * Sinv(:,:,kk) * zdiff ) * ...
                    %     GM_inten_in_prev(kk);
                    likelipf (1,kk) = mvnpdf(meas(:,jj),pred_z(:,kk),S(:,:,kk)) * GM_inten_in_prev(kk);
                    
                end
                likelipz(1,jj) = filter_params.clutter_intensity + sum (likelipf * filter_params.P_d,2) ;
            end
            %Parlikeli(1,par_ind) = (prod(likelipz,2) + 1e-99) *
            %cur_particle.w;  From Lin Gao code
            Parlikeli(1,par_ind) = exp(sum(GM_inten_in_prev,2)) * (prod(likelipz,2) + 1e-99) * cur_particle.w;
            
            %% Update missed detection terms
            GM_inten_in = (1 - filter_params.P_d) * GM_inten_in_prev;

            %% Update PHD component of detected 
            l = 0;
            for zz = 1:size(meas,2)
                l = l + 1;
                tau = zeros(1,num_GM_in);
                for jj = 1:num_GM_in
                    tau(1,jj) = filter_params.P_d * ...
                        GM_inten_in_prev(jj) * ...
                        mvnpdf(meas(:,zz),pred_z(:,jj),S(:,:,jj));
                    % zdiff = meas(:, zz) - pred_z(:,jj);
                    % tau(1,jj) = filter_params.P_d * ...
                    %     GM_inten_in_prev(jj) * ...
                    %     1 / sqrt(det(S(:,:,jj)) * (2*pi) ^size(filter_params.R,1))*...
                    %     exp(-0.5*(zdiff)' * Sinv(:,:,jj) * zdiff );
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
    exp_lm(i) = exp_num_landmark;
    num_lm_truth(i) = size(truth.cumulative_landmark_in_FOV{i,1},2);
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

    %% Resample particles (from Lin Gao)
    wei_ud = Parlikeli / sum(Parlikeli,2);
    for par_ind = 1:filter_params.num_particle
        particles(1,par_ind).w = wei_ud(par_ind);
    end
    est.num_effective_part(:,i) = 1/ sum(wei_ud.^2,2);
    if est.num_effective_part(:,i) < 0.2 * filter_params.num_particle
        dbg_str = sprintf('Num of effective particle is %d . Resample triggered', est.num_effective_part(:,i));
        disp (dbg_str)
        %resample_ind = particle_resample(wei_ud, filter_params.num_particle);
        resample_ind = low_variance_resample(wei_ud, filter_params.num_particle);
        for par_ind = 1:filter_params.num_particle
            particles(1,par_ind).pos = particles(1,resample_ind(1,par_ind)).pos;
            particles(1,par_ind).quat = particles(1,resample_ind(1,par_ind)).quat;
            particles(1,par_ind).gm_mu = particles(1,resample_ind(1,par_ind)).gm_mu;
            particles(1,par_ind).gm_inten = particles(1,resample_ind(1,par_ind)).gm_inten;
            particles(1,par_ind).gm_cov = particles(1,resample_ind(1,par_ind)).gm_cov;
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
        % for j=1:size(particles,2)
        %     scatter3(particles(j).pos(1), particles(j).pos(2), particles(j).pos(3),'r.');
        % end
        hold off
        %draw_particle_pos(particles,1)
        xlabel("X");
        ylabel("Y");
        zlabel("Z");
        
        % xlim ([min(truth.landmark_locations(1,:)), max(truth.landmark_locations(1,:))])
        % ylim([min(truth.landmark_locations(2,:)), max(truth.landmark_locations(2,:))])
        axis equal
        title_str = sprintf("Expected num of landmark = %d. t = %f", exp_num_landmark,time_vec(i));
        title(title_str)
        

        % figure(2)
        % draw_trajectory(truth.pos(:,i), truth.quat(i,:), [0;0;0], 1, 10, 2,'k',false);
        % hold on
        % draw_phd(max_likeli_gm_mu, max_likeli_gm_cov, max_likeli_gm_inten,[-30 150], truth.landmark_locations,"Test")
         %exportgraphics(fig2, "phd5.gif", Append=true);
        xlim([-10 120])
        ylim([-10 120])
        zlim([-10 10])
        %exportgraphics(fig1, "map.gif", Append=true);
        drawnow;
     end
     dbg_str = sprintf("timestep %f, exp num_landmark %d, num_landmark in FOV %d",...
         time_vec(i),exp_num_landmark,...
         size(truth.cumulative_landmark_in_FOV{i,1},2));
     disp(dbg_str);
end


%%
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
xlim ([min(truth.landmark_locations(1,:)), max(truth.landmark_locations(1,:))])
ylim([min(truth.landmark_locations(2,:)), max(truth.landmark_locations(2,:))])
zlim([-10 10])
axis equal
title_str = sprintf("Expected num of landmark = %d. is = %d", exp_num_landmark,i);
title(title_str)

figure(2)
plot (time_vec, num_lm_truth, 'DisplayName','truth');
hold on 
plot (time_vec, exp_lm,'DisplayName','est');
legend
