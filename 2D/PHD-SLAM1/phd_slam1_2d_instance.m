function results = phd_slam1_2d_instance(dataset, sensor_params, odom_params, filter_params, draw)
    addpath '../../util/'
    rng(420)
    time_vec = dataset.time_vec;
    dt = time_vec(2) - time_vec(1);

    if draw
        fig1 = figure(1);
        title ("Sim world")
        fig1.Position = [1,1,1000,1000];

        frame = cell(size(time_vec,2)-1,1);
    end

    %% Prepare truth data struct
    truth.pos = dataset.pos;
    truth.quat = dataset.quat;
    
    % Prepare some odom estimation
    odom.pos = truth.pos;
    odom.quat = truth.quat;
    odom.body_trans_vel = zeros(3,size(time_vec,2));
    odom.body_rot_vel = odom.body_trans_vel;

    % Pre run the sim to generate map and mease data. Should help with run 
    % time as well
    truth.cummulative_landmark_in_FOV = cell(size(time_vec,2),1);
    meas_table = cell(size(time_vec,2),1);
    disp('Pre generate odom est, measurements and cummulative landmark map')
    for kk = 1:size(time_vec,2)
        [cur_meas, ~,landmark_in_FOV] = gen_meas_cartesian_2D(truth.pos(:,kk),...
            truth.quat(kk,:),dataset.landmark_locations, sensor_params);

        meas_table{kk,:} = cur_meas;
        if kk == 1
            truth.cummulative_landmark_in_FOV{kk,1} = landmark_in_FOV;
        else
            temp = unique(vertcat(truth.cummulative_landmark_in_FOV{kk-1,1}(:,:)', landmark_in_FOV'),'rows');
            truth.cummulative_landmark_in_FOV{kk,1} = temp';
        end
        
        % Simulated pure odometry
        if kk == 1
            odom.pos(:,kk) = truth.pos(:,kk);
            odom.quat(kk,:) = truth.quat(kk,:);
        else
            % Sample odometry - constraint to 2D
            body_trans_vel_sample = zeros(3,1);
            body_rot_vel_sample = zeros(3,1);
    
            body_trans_vel_sample(1,1) = normrnd(0,odom_params.motion_sigma(1));
            body_trans_vel_sample(2,1) = normrnd(0,odom_params.motion_sigma(2));
            body_rot_vel_sample(3,1) = normrnd(0,odom_params.motion_sigma(3));
    
            body_trans_vel = dataset.trans_vel_body(:,kk) + body_trans_vel_sample;
            body_rot_vel = dataset.rot_vel_body(:,kk) + body_rot_vel_sample;

            odom.body_trans_vel(:,kk) = body_trans_vel;
            odom.body_rot_vel(:,kk) = body_rot_vel;

            [odom.pos(:,kk), odom.quat(kk,:)] = ...
                propagate_state (odom.pos(:,kk-1), odom.quat(kk-1,:),...
                body_trans_vel, body_rot_vel, dt);
        end
    end
    disp('done')
    clc;

    %% Pre allocate datas for estimation

    est.pos = truth.pos;
    est.quat = truth.quat;
    est.map_est = cell(size(time_vec,2),1);
    est.compute_time = zeros(size(time_vec,2),1);
    
    %% Initialize filter
    if strcmp(sensor_params.meas_model,'cartesian')
        cur_meas = meas_table{1,1};

        meas_world_frame = reproject_meas(truth.pos(:,1),truth.quat(1,:),cur_meas, sensor_params);
    else
        error_msg = strcat(sensor_params.meas_model, " measurement model is not supported");
        error(error_msg);
    end

    particles = init_phd1_particles_2D(filter_params.num_particle, ...
        truth.pos(:,1),truth.quat(1,:),meas_world_frame,filter_params.birthGM_cov, filter_params.birthGM_intensity);

    %% Run simulation
    for kk = 2:size(time_vec,2) 
        %% Get current measurements and reproject for viz
        cur_meas = meas_table{kk,1};
        meas_reprojected = reproject_meas(truth.pos(:,kk), truth.quat(kk,:),...
            cur_meas, sensor_params);

        out_loop_timer = tic;
        for par_ind = 1:size(particles,2)
            cur_par = particles(par_ind);

            %% Particle time update
            if strcmp(filter_params.motion_model,'odometry')
                % Sample odometry - constraint to 2D
                body_trans_vel_sample = zeros(3,1);
                body_rot_vel_sample = zeros(3,1);
    
                body_trans_vel_sample(1,1) = normrnd(0,filter_params.motion_sigma(1));
                body_trans_vel_sample(2,1) = normrnd(0,filter_params.motion_sigma(2));
                body_rot_vel_sample(3,1) = normrnd(0,filter_params.motion_sigma(3));

                % Add noise to odom measurement
                body_trans_vel = odom.body_trans_vel(:,kk) + body_trans_vel_sample;
                body_rot_vel = odom.body_rot_vel(:,kk) + body_rot_vel_sample;
                
                [cur_pos, cur_quat] = ...
                    propagate_state (particles(1,par_ind).pos, particles(1,par_ind).quat,...
                    body_trans_vel, body_rot_vel, dt);
            elseif strcmp(filter_params.motion_model,'truth')
                cur_pos = truth.pos(:,kk);
                cur_quat = truth.quat(kk,:);
                
            else
                error_msg = strcat(filter_params.motion_model, " motion model is not supported");
                error(error_msg);
            end
            
            particles(1,par_ind).pos = cur_pos;
            particles(1,par_ind).quat = cur_quat;

            %% GM component checking step
            % Check for GM in FOV
            num_GM_prev = size(particles(1,par_ind).gm_mu,2);
            gm_mu_temp = vertcat(particles(1,par_ind).gm_mu,zeros(1,num_GM_prev));
            [~,GM_in_FOV] = check_in_FOV_2D(gm_mu_temp, ...
                particles(1,par_ind).pos, particles(1,par_ind).quat, sensor_params);

             % Extract GM components not in FOV. No changes are made to them
            GM_out_FOV = ~GM_in_FOV;
            GM_mu_out = particles(1,par_ind).gm_mu(:,GM_out_FOV);
            GM_cov_out = particles(1,par_ind).gm_cov (:,:,GM_out_FOV);
            GM_inten_out = particles(1,par_ind).gm_inten(GM_out_FOV);
    
            % Extract GM components in FOV. These are used during update
            % Predict 
            GM_mu_in = particles(1,par_ind).gm_mu(:,GM_in_FOV);
            GM_cov_in = particles(1,par_ind).gm_cov (:,:,GM_in_FOV);
            GM_inten_in = particles(1,par_ind).gm_inten(GM_in_FOV);
            
            num_GM_in = size(GM_inten_in,2);

            %% Per particle map update
            % Only do update if there are GM in the FOV
            if num_GM_in > 0
                for jj = 1:num_GM_in
                    GM_cov_in(:,:,jj) = GM_cov_in(:,:,jj) + filter_params.map_Q;
                end

                [particles(1,par_ind).w, GM_mu_in, GM_cov_in, GM_inten_in]=...
                    phd_measurement_update(particles(1,par_ind),...
                    GM_mu_in, GM_cov_in, GM_inten_in, cur_meas, filter_params);

                %% Clean up GM components
                [GM_mu_in, GM_cov_in, GM_inten_in] = cleanup_PHD (GM_mu_in,...
                GM_cov_in, GM_inten_in, filter_params.pruning_thres, ...
                filter_params.merge_dist, filter_params.num_GM_cap);

                %% Parse updated GM and include out of FOV components
                particles(1,par_ind).gm_mu = cat(2,GM_mu_in, GM_mu_out);
                particles(1,par_ind).gm_inten = cat (2, GM_inten_in, GM_inten_out);
                particles(1,par_ind).gm_cov = cat(3,GM_cov_in, GM_cov_out);

            else%num_GM_in > 0
                particles(1,par_ind).w = 1e-99;
            end %num_GM_in > 0
        end %par_ind = 1:size(particles,2)

        %% State estimation
        [pose_est, map_est_struct] = extract_estimates_max_likeli(particles, filter_params);
        est.pos(:,kk) = pose_est.pos;
        est.quat(kk,:) = pose_est.quat;
        % Add zero z component for map
        map_est = vertcat(map_est_struct.feature_pos,zeros(1,size(map_est_struct.feature_pos,2)));
        est.map{kk,1} = map_est;

        % Adaptive birth PHD (Lin Gao's implementation)
        particles = adaptive_birth_PHD_2D (pose_est.pos, pose_est.quat, cur_meas, map_est_struct, filter_params, particles);
        
        % Resample (if needed)
        if mod(kk,20) == 0
            disp("check resample")
            [particles, est.num_effective_particle(kk)] = resample_particles(particles, filter_params);
        end

        % Timing
        est.compute_time(kk) = toc(out_loop_timer);
    
        if draw
        %%Ploting
        figure(1)
        draw_trajectory(truth.pos(:,kk), truth.quat(kk,:), truth.pos(:,1:kk),4, 2,'k',false);
        draw_trajectory(est.pos(:,kk), est.quat(kk,:), est.pos(:,1:kk), 4, 2, 'g',true);
        draw_trajectory(odom.pos(:,kk), odom.quat(kk,:), odom.pos(:,1:kk), 4, 2, 'r',true);
        hold on
        set(gca, 'Zdir', 'reverse')
        set(gca, 'Ydir', 'reverse')
        grid on
        scatter3(truth.cummulative_landmark_in_FOV{end,1}(1,:),...
            truth.cummulative_landmark_in_FOV{end,1}(2,:),...
            truth.cummulative_landmark_in_FOV{end,1}(3,:),...
            ones(size(truth.cummulative_landmark_in_FOV{end,1},2),1) * 50,'k')
        scatter3(meas_reprojected(1,:), meas_reprojected(2,:), meas_reprojected(3,:),...
            ones(size(meas_reprojected,2),1) * 50,'b*');
        scatter3(map_est(1,:), map_est(2,:), map_est(3,:),...
            ones(size(map_est,2),1) * 50,'r+')
        xlabel("X (m)");
        ylabel("Y (m)");
        zlabel("Z (m)");
        %axis equal;
        xlim([min(truth.cummulative_landmark_in_FOV{end,1}(1,:) - 10), max(truth.cummulative_landmark_in_FOV{end,1}(1,:) + 10)])
        ylim([min(truth.cummulative_landmark_in_FOV{end,1}(2,:) - 10), max(truth.cummulative_landmark_in_FOV{end,1}(2,:) + 10)])
        title_str = sprintf("Index = %d. t = %f", kk,time_vec(kk));
        plot_2D_phd(map_est_struct,500,0,1)
        colorbar
        title(title_str)
        view(0,90)
        drawnow
        frame{kk-1} = getframe(gcf);
        end %draw
        

    end %kk = 2:size(time_vec,2)
    
    if draw
        % Write video
        obj = VideoWriter("myvideo");
        obj.FrameRate = 20;
        open(obj);
        for i=1:length(frame)
            writeVideo(obj,frame{i})
        end
        obj.close();
    end
    % End simulation
    results.truth = truth;
    results.filter_est = est;
    results.odom_est = odom;
    results.time_vec = time_vec;


end