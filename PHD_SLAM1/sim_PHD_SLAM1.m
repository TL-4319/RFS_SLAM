% Simulation. The math is general for 3D but the simulation is limit to 2D
close all;
clear;
clc;

%% Load truth and measurement data
addpath ('../util/')
load('../dataset/truth_2.mat');
load('../dataset/meas_table_2.mat');
truth_hist = truth.pos(:,1);

%% 
rng(42069);
time_vec = truth.time_vec;
dt = time_vec(2) - time_vec(1);
draw = false;

%% Drawing stuffs
if draw
    fig1 = figure(1);
    title ("Sim world")
    fig1.Position = [1,1,600,600];
    
    fig2 = figure(2);
    title("Sensor frame")
    fig2.Position = [611,1,600,600];
end


%% Prealocate estimated traj
est_traj = truth;
est_traj.hist = truth.pos(:,1);

%% SLAM configuration
% Trajectory config
num_particle = 1000;
% Motion covariance = [cov_x, cov_y, cov_z, cov_phi, cov_theta, cov_psi]
% Use 3D navigator motion model. z, phi, theta are 0 to maintain 2D for now
motion_sigma = [5; 5; 0; 0; 0; 2];

% Map PHD config
birthGM_intensity = 0.1;
birthGM_cov = [0.2, 0, 0; 0, 0.2, 0; 0, 0, 0.0001];

% Sensor model
map_Q = diag([0.2, 0.2, 0.001]);
filter_sensor_noise = 0.2;
R = diag([filter_sensor_noise^2, filter_sensor_noise^2, 0.00001]);
%clutter_intensity = sensor.clutter_rate / (sensor.Range^2 * sensor.HFOV * 0.5) * 1e-4;
clutter_intensity = 50 / (20^2 * 0.3 * pi);
P_d = 0.7;

% PHD management parameters
pruning_thres = 10^-5;
merge_dist = 4;
num_GM_cap = 100;

landmark_threshold = 0.01;
%% Initialize SLAM particles
cur_pos = est_traj.pos(:,1);
cur_quat = est_traj.quat(1,:);
meas = meas_table{1,1};
meas_world_frame = reproject_meas(cur_pos, cur_quat, meas);
particles = init_phd_particles (num_particle, cur_pos, cur_quat, meas_world_frame, birthGM_cov, birthGM_intensity);

%% Run simulation
for i=2:size(time_vec,2)
    disp (time_vec(i));
    % map_png_name = sprintf ("/home/tuan/Projects/anarsh/visual_RB_PHD_SLAM/map/%s.png",pad(sprintf('%d',i), 3, 'left','0'));
    % figure(1)
    % imshow(map_png_name);

    %% Parse some truth data
    truth_hist = horzcat (truth_hist, truth.pos(:,i));
    meas = meas_table{i,1};

    meas_reprojected = reproject_meas(truth.pos(:,i), truth.quat(i,:), meas);

    pause_ind = -1;
    if i == pause_ind + 1
        disp("Pause");
    end

    %% SLAM runs here
    Parlikeli = zeros (1,num_particle);
    for par_ind = 1:size(particles,2)
        cur_particle = particles(1,par_ind);
        %% Trajectory prediction
        body_vel_sample = randn(3,1) .* motion_sigma(1:3);
        body_rot_vel_sample = randn(3,1) .* motion_sigma(4:6);
        
        % Constraint vel to 2D
        body_vel_sample(3,:) = 0;
        body_rot_vel(1:2,:) = zeros (2,1);
        

        [cur_particle.pos, cur_particle.quat] = propagate_state (cur_particle.pos, ...
            cur_particle.quat,body_vel_sample, body_rot_vel_sample, dt);

        % Give particle truth pose for map debug
        %cur_particle.pos = truth.pos(:,i);
        %cur_particle.quat = truth.quat(i,:);

        particles(1,par_ind).pos = cur_particle.pos;
        particles(1,par_ind).quat = cur_particle.quat;
    

        % figure(2)
        % draw_trajectory(truth.pos(:,i), truth.quat(i,:), [0;0;0], 1, 10, 2,'k',false);
        % str_name = sprintf("All GM at %d", i);
        % hold on
        % draw_phd(cur_particle.gm_mu, cur_particle.gm_cov, cur_particle.gm_inten,[-30 150], truth.landmark_locations,str_name)
         

        %% Check if GM is in FOV
        num_GM = size(cur_particle.gm_mu,2);
        isinFOV = zeros (1, num_GM);
        for kk = 1:num_GM
            isinFOV(kk) = check_in_FOV (cur_particle.gm_mu(:,kk), ...
                cur_particle.pos, cur_particle.quat, truth.sensor);
        end

        %% Extract GM components not in FOV. No changes are made to them
        GM_out_FOV = find (~isinFOV);
        GM_mu_out = cur_particle.gm_mu(:,GM_out_FOV);
        GM_cov_out = cur_particle.gm_cov (:,:,GM_out_FOV);
        GM_inten_out = cur_particle.gm_inten(GM_out_FOV);

        %% Extract GM components in FOV. These are used during update
        % Predict 
        GM_in_FOV = find(isinFOV);
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
                GM_cov_in(:,:,kk) = GM_cov_in_prev(:,:,kk) + map_Q;
            end


            % Compute particle likelihood - NEED MORE RESEARCH
            likelipz = zeros (1,size(meas,2));
            for jj = 1:size(meas,2)
                likelipf = zeros (1,num_GM_in);
                for kk = 1:num_GM_in
                    pred_z = gen_pred_meas(cur_particle.pos, cur_particle.quat, GM_mu_in_prev(:,kk));
                    zdiff = meas(:, jj) - pred_z;
                    %%%%%
                    likelipf (1,kk) = 1/sqrt(det(2*pi*R))*exp(-0.5*(zdiff)'*inv(R)*zdiff ) * GM_inten_in_prev(kk);
                    %likelipf (1, kk) = mvnpdf(meas(:, jj), pred_z, R) * clutter_intensity;
                end
                likelipz(1,jj) = clutter_intensity + sum (likelipf,2);
            end
            Parlikeli(1,par_ind) = prod(likelipz,2) + 1e-99;


            % Pre compute inner update terms
            [pred_z, K, S, P] = compute_update_terms (cur_particle, GM_mu_in_prev, GM_cov_in_prev, R);

            % Update missed detection terms
            GM_inten_in = (1 - P_d) * GM_inten_in_prev;

            %Update PHD component of detected 
            l = 0;
            for zz = 1:size(meas,2)
                l = l + 1;
                tau = zeros(1,num_GM_in);
                for jj = 1:num_GM_in
                    tau(1,jj) = P_d * GM_inten_in_prev(jj) * mvnpdf(meas(:,zz),pred_z(:,jj),S(:,:,jj));
                    mu = GM_mu_in_prev(:,jj) + K(:,:,jj)* (meas(:,zz) - pred_z(:,jj));
                    GM_mu_in = horzcat(GM_mu_in, mu);
                    GM_cov_in = cat(3,GM_cov_in, P(:,:,jj));
                end
                    tau_sum = sum(tau);
                for jj = 1:num_GM_in
                    nu = tau(jj) / (clutter_intensity + tau_sum);
                    GM_inten_in = horzcat(GM_inten_in, nu);
                end
            end
            %% Clean up GM components
            % Prune
            [GM_mu_in, GM_cov_in, GM_inten_in] = cleanup_PHD (GM_mu_in, GM_cov_in, GM_inten_in, pruning_thres, merge_dist, num_GM_cap);
    
            %% Add back components not in FOV
            particles(1,par_ind).gm_mu = cat(2,GM_mu_in, GM_mu_out);
            particles(1,par_ind).gm_inten = cat (2, GM_inten_in, GM_inten_out);
            particles(1,par_ind).gm_cov = cat(3,GM_cov_in, GM_cov_out);
        %% If there are no map GM in FOV of particle
        else
            particles(1,par_ind).w = 1e-99;
        end

    end
    %% State estimation (max likelihood)
    % Trajectory estimation
    [max_likeli, max_w_particle_ind] = max(Parlikeli);

    % MAP trajectory estimation
    est_traj.pos = particles(1,max_w_particle_ind).pos;
    est_traj.quat = particles(1,max_w_particle_ind).quat;
    est_traj.hist = horzcat(est_traj.hist, est_traj.pos);
    
    % MAP Landmark estimation
    max_likeli_gm_mu = particles(1,max_w_particle_ind).gm_mu;
    max_likeli_gm_inten = particles(1,max_w_particle_ind).gm_inten;
    max_likeli_gm_cov = particles(1,max_w_particle_ind).gm_cov;
    % Find expected number of landmark
    exp_num_landmark = round(sum (max_likeli_gm_inten));
    %ID_map = find(max_likeli_gm_inten > landmark_threshold);
    [~,ID_map] = maxk (max_likeli_gm_inten, exp_num_landmark);
    map_est = max_likeli_gm_mu(:,ID_map);

    %% Adaptive birth PHD (from Lin Gao paper)
    new_birth_mu = []; new_birth_inten = []; new_birth_cov = [];
    new_meas_world_frame = reproject_meas(est_traj.pos, est_traj.quat, meas);
    for zz = 1:size(meas,2)
        cur_GM_mu = particles(1,max_w_particle_ind).gm_mu;
        matrix_dist = repmat(new_meas_world_frame(:,zz),1, size(cur_GM_mu,2)) - cur_GM_mu;
        dist = vecnorm(matrix_dist);
        if min(dist) >= 5 % If the measurement is not close to any existing landmark/target
            new_birth_inten = horzcat (new_birth_inten,birthGM_intensity);
            new_birth_mu = cat (2,new_birth_mu, new_meas_world_frame(:,zz));
            new_birth_cov = cat (3, new_birth_cov, birthGM_cov);
        end
    end
    for par_ind = 1:size(particles,2)
        particles(1,par_ind).gm_cov = cat(3,particles(1,par_ind).gm_cov, new_birth_cov);
        particles(1,par_ind).gm_inten = horzcat(particles(1,par_ind).gm_inten, new_birth_inten);
        particles(1,par_ind).gm_mu = horzcat(particles(1,par_ind).gm_mu, new_birth_mu);
    end

    %% Resample particles (from Lin Gao)
    wei_ud = Parlikeli / sum(Parlikeli,2);
    resample_ind = particle_resample(wei_ud, num_particle);
    for par_ind = 1:num_particle
        particles(1,par_ind).pos = particles(1,resample_ind(1,par_ind)).pos;
        particles(1,par_ind).quat = particles(1,resample_ind(1,par_ind)).quat;
        particles(1,par_ind).w = particles(1,resample_ind(1,par_ind)).w;
        particles(1,par_ind).gm_mu = particles(1,resample_ind(1,par_ind)).gm_mu;
        particles(1,par_ind).gm_inten = particles(1,resample_ind(1,par_ind)).gm_inten;
        particles(1,par_ind).gm_cov = particles(1,resample_ind(1,par_ind)).gm_cov;
    end
    %% Plotting

     if draw
        % Plotting
        figure(1)
        draw_trajectory(truth.pos(:,i), truth.quat(i,:), truth_hist, 1, 10, 2,'k',false);
        draw_trajectory(est_traj.pos, est_traj.quat, est_traj.hist, 1, 10, 2,'g',true);
        hold on
        set(gca, 'Zdir', 'reverse')
        set(gca, 'Ydir', 'reverse')
        grid on
        view([0,90])
        
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
        exportgraphics(fig1, "map5.gif", Append=true);

        figure(2)
        draw_trajectory(truth.pos(:,i), truth.quat(i,:), [0;0;0], 1, 10, 2,'k',false);
        hold on
        draw_phd(max_likeli_gm_mu, max_likeli_gm_cov, max_likeli_gm_inten,[-30 150], truth.landmark_locations,"Test")
        %exportgraphics(fig2, "phd5.gif", Append=true);
        drawnow;
    end
end

pos_dif = est_traj.hist - truth_hist;
pos_dif_sq = pos_dif.^2;
dist_error = pos_dif_sq(1,:) + pos_dif_sq(2,:);
dist_error = dist_error.^0.5;

figure(3)
plot (time_vec, dist_error);
name = sprintf("%d Particles", num_particle);
title(name)
grid on;
xlabel ("time(s)");
ylabel ("Dist error (m)");