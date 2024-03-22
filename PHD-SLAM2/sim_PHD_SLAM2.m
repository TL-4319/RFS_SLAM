%Simulation. Math is general for 3D but the simulation is limit to 2D
close all;
clear;
clc;

%%
rng(420);
draw = false;

%% Load truth and measurement data
addpath ('../util/')

load('../dataset/truth_2D_2.mat');

%% Time vector 
time_vec = truth.time_vec;
dt = time_vec(2) - time_vec(1);

%% Drawing stuffs
if draw
    fig1 = figure(1);
    title ("Sim world")
    fig1.Position = [1,1,800,800];
    
    fig2 = figure(2);
    title("Sensor frame")
    fig2.Position = [611,1,600,600];
end

%% Prealocate estimated traj
est = truth;
est.map_est = cell(size(time_vec,2),1);
est.compute_time = zeros (1,size(time_vec,2));
est.num_effective_part = est.compute_time;

%% Odometry configuration
%% Odometry parameters
odom.sigma_trans = [0.5; 0.5; 0.3];
odom.sigma_rot = [0.01; 0.01; 0.2];
odom.body_trans_vel = truth.body_trans_vel + ...
    randn(3,size(truth.body_trans_vel,2)) .* repmat(odom.sigma_trans,1,size(truth.body_trans_vel,2));
odom.body_rot_vel = truth.body_rot_vel + ...
    randn(3,size(truth.body_rot_vel,2)) .* repmat(odom.sigma_rot,1,size(truth.body_rot_vel,2));

% Constraint odom to 2D
odom.body_trans_vel(3,:) = zeros(1,size(odom.body_trans_vel,2));
odom.body_rot_vel(1:2,:) = zeros(2,size(odom.body_trans_vel,2));

%% SLAM configuration
filter_params.resample_scheme = 1; % 0 is no resampling, 1 is resample at every step, 2 is adaptive resample
filter_params.resample_trigger = 0;
filter_params.num_particle = 1;
filter_params.intial_particle_cov = diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.001]).^2;
filter_params.process_noise = diag([0.6 0.6 0.00001 0.00001 0.000001 0.3]).^2;

% Map PHD config
filter_params.birthGM_intensity = 0.1;
filter_params.birthGM_cov = [0.2, 0, 0; 0, 0.2, 0; 0, 0, 0.0001];

% Sensor model
filter_params.map_Q = diag([0.2, 0.2, 0.001]);
filter_params.filter_sensor_noise = 0.1;
filter_params.R = diag([filter_params.filter_sensor_noise^2, ...
    filter_params.filter_sensor_noise^2, 0.00001]);
%clutter_intensity = sensor.clutter_rate / (sensor.Range^2 * sensor.HFOV * 0.5) * 1e-4;
filter_params.clutter_intensity = 50 / (20^2 * 0.3 * pi);
filter_params.P_d = 0.9;

% PHD management parameters
filter_params.pruning_thres = 10^-5;
filter_params.merge_dist = 4;
filter_params.num_GM_cap = 3000;

%% Initialize SLAM particles
cur_pos = est.pos(:,1);
cur_quat = est.quat(1,:);
meas = truth.meas_table{1,1};
meas_world_frame = reproject_meas(cur_pos, cur_quat, meas);
particles = init_phd_particles (filter_params.num_particle, cur_pos, cur_quat, ...
    meas_world_frame, filter_params.birthGM_cov, filter_params.birthGM_intensity, filter_params.intial_particle_cov);

%% Run simulation
for i = 2:size(time_vec,2)
    

    %% Parse some truth data
    meas = truth.meas_table{i,1};

    meas_reprojected = reproject_meas(truth.pos(:,i), truth.quat(i,:), meas);

    %% SLAM runs here
    Parlikeli = zeros (1,filter_params.num_particle);

    for par_ind = 1:filter_params.num_particle
        particles(par_ind).P = filter_params.intial_particle_cov;
        
        %% Propagation
        cur_particle = particles(1,par_ind);
        trans_odom = odom.body_trans_vel(:,i);
        rot_odom = odom.body_rot_vel(:,i);
        
        %Propagate particle state and covariance
        [F, Q] = calc_pred_jacobian(cur_particle.pos, compact(cur_particle.quat),...
            trans_odom, rot_odom, dt);

        [cur_particle.pos, cur_particle.quat] = propagate_state(cur_particle.pos, cur_particle.quat,...
            trans_odom, rot_odom, dt);
        cur_particle.P = F * cur_particle.P * F' + Q * filter_params.process_noise * Q';
        pred_traj = [cur_particle.pos; compact(cur_particle.quat)'];
        pred_P = cur_particle.P;

        %% Map prediction
        % Check for GM in FOV
        [~, GM_in_FOV] = check_in_FOV_2D (cur_particle.gm_mu, ...
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

        num_GM = size(GM_inten_in,2);
        % Augment to include particle state
        aug_inten_in = GM_inten_in;
        aug_mu_in = cat(1,GM_mu_in, repmat([cur_particle.pos; compact(cur_particle.quat)'],1,num_GM));
        aug_cov = zeros(10,10,num_GM);
        for j = 1:num_GM
            aug_cov(:,:,j) = blkdiag(GM_cov_in(:,:,j), cur_particle.P);
        end

        %% Measurement update only perform if there are GM in FOV
        if num_GM > 0 || size(meas,2) == 0
            %% Importance sampling
            % Static map selection
            z_pred = zeros(3,num_GM);
            HH = zeros(3,10,num_GM);
            SS = zeros(3,3, num_GM);
            KK = zeros(10,3,num_GM);
            cov_update = zeros(10,10,num_GM);
            for j = 1:num_GM
                landmark_pos = GM_mu_in(:,j);
                HH(:,:,j) = calc_meas_jacobian(cur_particle.pos, compact(cur_particle.quat),landmark_pos);
                SS(:,:,j) = HH(:,:,j) * aug_cov(:,:,j) * HH(:,:,j)' + filter_params.R;
                KK(:,:,j) = aug_cov(:,:,j) * HH(:,:,j)' * pinv(SS(:,:,j)); 
                z_pred(:,j) = gen_pred_meas(cur_particle.pos, cur_particle.quat,GM_mu_in(:,j));
                cov_update(:,:,j) = (eye(10) - KK(:,:,j) * HH(:,:,j)) * aug_cov(:,:,j);
            end
            
            % Update static map
            likeliall = zeros(1,size(meas,2));
            inten_upd = zeros(1,0);
            mu_up = zeros(10,0);
            cov_up = zeros(10,10,0);
            % Itereate over all association
            for j = 1:size(meas,2)
                likeligm = zeros(1,num_GM); 
                zreal = meas(:,j);
                for k = 1:num_GM
                    zdiff = zreal - z_pred(:,k);
                    likeligm(1,k) = filter_params.P_d * (det(2 * pi * SS(:,:,k)))^-1 * ...
                        exp(-0.5 * zdiff' * pinv(SS(:,:,k)) * zdiff) * aug_inten_in(1,k) + 1e-299; % Likelihood that GM(k) corresponds to obs(j)
                    inten_upd  = cat(2,inten_upd, likeligm(1,k));
                    cov_up = cat(3, cov_up, cov_update(:,:,k));
                    mu_up = cat(2, mu_up, aug_mu_in(:,k) + KK(:,:,k) * zdiff);
                end
                likeliall(1,j) = sum(likeligm,2);
            end
            likelicomb = sum(likeliall,1);

            % Compute final PHD
            for j = 1:size(meas,2)
                Id = (j-1)*num_GM + 1: j * num_GM;
                test = inten_upd(1,Id)./(likelicomb(1,j) + filter_params.clutter_intensity);
                inten_upd(1,Id) = inten_upd(1,Id)./(likelicomb(1,j) + filter_params.clutter_intensity);
            end

            % Separate landmark and traj
            weitraj = inten_upd / sum(inten_upd,2); % normalize
            trajup = mu_up(4:10,:);
            covtraj = zeros(7,7,size(cov_up,3));
            for j = 1:size(cov_up,3)
                covtraj(:,:,j) = cov_up(4:10,4:10,j);
            end
            Idw = find(inten_upd >= 0.5);

            % Discarding first part phd
            weit = weitraj(1,Idw)/ sum(weitraj(1,Idw),2);
            trajup = trajup (:,Idw);
            covtraj = covtraj(:,:,Idw);
            
            OMEGAtraj = zeros(7,7,size(Idw,2)); 
            qup = zeros(7,size(Idw,2)); % IS THIS PARAM NEEDED  

            for j = 1:size(Idw,2)
                OMEGAtraj(:,:,j) = weit(1,j) * pinv(covtraj(:,:,j));
                qup(:,j) = weit(1,j) * pinv(covtraj(:,:,j)) * trajup(:,j);
            end
            % Estimated trajectory and cov matrix
            trajest = sum(repmat(weit,7,1).* trajup, 2); % Weighted sum 
            Pup = zeros(7,7);
            for j = 1:size(qup,2)
                Pup = Pup + weit(:,j) * (covtraj(:,:,j) + ...
                    (trajest - trajup(:,j)) * (trajest - trajup(:,j))');
            end
            
            % Importance sampling based on measurement update
            if isempty(Idw) ~= 1
                Markss = 1;
                while (Markss)
                    cur_particle.P = Pup;
                    temp_traj = trajest + chol(Pup) * randn(7,1);
                    cur_particle.pos = temp_traj(1:3,1);
                    cur_particle.quat = quaternion(temp_traj(4,1),...
                        temp_traj(5,1), temp_traj(6,1), temp_traj(7,1));

                    % Constraint to 2D
                    cur_particle.pos(3) = 0;
                    euler = quat2eul(cur_particle.quat);
                    euler(2:3) = zeros(1,2);
                    temp_quat = eul2quat(euler);
                    cur_particle.quat = quaternion(temp_quat(1,1),...
                        temp_quat(1,2), temp_quat(1,3), temp_quat(1,4));

                    wei_importance = 1/ sqrt(det(2*pi*pred_P)) * exp(-0.5 * (temp_traj - pred_traj)' *...
                        pinv(pred_P) * (temp_traj - pred_traj)) / ...
                        (1/sqrt(det(2 * pi * Pup)) * exp(-0.5 * (temp_traj-trajest)' * ...
                        pinv(Pup) * (temp_traj - trajest)));
                    if isnan(wei_importance)
                        Markss = 1;
                    else 
                        Markss = 0;
                    end
                end

            else
                wei_importance = 1;
            end
            particles(par_ind) = cur_particle;

            %% Normal PHD-SLAM1 update based on new proposal trajectory
            GM_cov_in_prev = GM_cov_in;
            GM_mu_in_prev = GM_mu_in;
            GM_inten_in_prev = GM_inten_in;

            for kk = 1:num_GM
                GM_cov_in(:,:,kk) = GM_cov_in_prev(:,:,kk) + filter_params.map_Q;
            end

            %% Compute particle using single cluster likelihood
            % Assign likelihood based on best association
            likelipz = zeros(1,size(meas,2));
            for jj = 1:size(meas,2)
                likelipf = zeros(1,num_GM);
                for kk = 1:num_GM
                    pred_z = gen_pred_meas(cur_particle.pos, cur_particle.quat, GM_mu_in_prev(:,kk));
                    zdiff = meas(:,jj) - pred_z;
                    likelipf (1,kk) = 1/sqrt(det(2*pi*filter_params.R))*...
                        exp(-0.5*(zdiff)'*pinv(filter_params.R)*zdiff ) * ...
                        GM_inten_in_prev(kk);
                end
                likelipz(1,jj) = filter_params.clutter_intensity + sum(likelipf,2);
            end
            Parlikeli(1,par_ind) = prod(likelipz,2) + 1e-99;
            %% Map update. Follow GM-PHD process
            %% Pre compute inner update terms
            [pred_z, K, S, P] = compute_update_terms (cur_particle,...
                GM_mu_in_prev, GM_cov_in_prev, filter_params.R);

            % Update missed detection terms
            GM_inten_in = (1 - filter_params.P_d) * GM_inten_in_prev;
            
            %Update PHD component of detected GM 
            l = 0;
            for zz = 1:size(meas,2)
                l = l + 1;
                tau = zeros(1,num_GM);
                for jj = 1:num_GM
                    tau(1,jj) = filter_params.P_d * ...
                        GM_inten_in_prev(jj) * ...
                        mvnpdf(meas(:,zz),pred_z(:,jj),S(:,:,jj));
                    mu = GM_mu_in_prev(:,jj) + K(:,:,jj)* (meas(:,zz) - pred_z(:,jj));
                    GM_mu_in = horzcat(GM_mu_in, mu);
                    GM_cov_in = cat(3,GM_cov_in, P(:,:,jj));
                end
                    tau_sum = sum(tau);
                for jj = 1:num_GM
                    nu = tau(jj) / (filter_params.clutter_intensity + tau_sum);
                    GM_inten_in = horzcat(GM_inten_in, nu);
                end
            end

            %% Clean up GM components with pruning, merge and capping 
            [GM_mu_in, GM_cov_in, GM_inten_in] = cleanup_PHD (GM_mu_in,...
                GM_cov_in, GM_inten_in, filter_params.pruning_thres, ...
                filter_params.merge_dist, filter_params.num_GM_cap);
            
            %% Add back components not in FOV
            particles(par_ind).gm_mu = cat(2,GM_mu_in, GM_mu_out);
            particles(par_ind).gm_inten = cat (2, GM_inten_in, GM_inten_out);
            particles(par_ind).gm_cov = cat(3,GM_cov_in, GM_cov_out);

        else % If no GM in FOV, reduce likelihood
            Parlikeli(1,i) = 1e-99;
        end

    end %END OF EACH PARTICLE

    %% State estimation (max likelihood)
    % Trajectory estimation
    [max_likeli, max_w_particle_ind] = max(Parlikeli);
    
    % re normalize weight
    norm_weight = Parlikeli / sum(Parlikeli,2);
    norm_weight_sq = norm_weight.^2;
    est.num_effective_part(1,i) = 1 / sum(norm_weight_sq);

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
    % Generate new birth GM component from previus measurements that is not
    % near a surviving GM component
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
    if filter_params.resample_scheme == 1
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
    elseif filter_params.resample_scheme == 2 && est.num_effective_part(1,i) < filter_params.resample_trigger
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
    else
        for par_ind = 1:filter_params.num_particle
            particles(par_ind).w = norm_weight(1,par_ind);
        end
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
        exportgraphics(fig1, "map.gif", Append=true);

        figure(2)
        draw_trajectory(truth.pos(:,i), truth.quat(i,:), [0;0;0], 1, 10, 2,'k',false);
        hold on
        draw_phd(max_likeli_gm_mu, max_likeli_gm_cov, max_likeli_gm_inten,[-50 250], truth.landmark_locations,"Test")
        %exportgraphics(fig2, "phd5.gif", Append=true);
        drawnow;
     end
     disp(time_vec(i));
end % END OF EACH TIMESTEP

%% Extract all data to one structure for saving
simulation.truth = truth;
simulation.odom = odom;
simulation.est = est;
simulation.filter_params = filter_params;