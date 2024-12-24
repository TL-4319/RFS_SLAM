function particle = adaptive_birth_CPHD_3D (base_pos, base_quat, measurement,...
    current_map_est, filter, prev_particle)
    %% Generate GM component from measurements if far enough from a previously mapped GM component
    particle = prev_particle; % Pre-allocating

    new_birth_mu = []; new_birth_inten = []; new_birth_cov = [];

    % Calc sensor pos 
    [sensor_pos, sensor_quat] = get_sensor_pose(base_pos, base_quat, filter.sensor);
    new_meas_world_frame = reproject_meas(sensor_pos, sensor_quat, measurement,filter.sensor);
    cur_GM_mu = current_map_est.max_likeli_gm_mu;
    if size(cur_GM_mu,2) ~= 0
        for zz = 1:size(measurement,2)
            matrix_dist = repmat(new_meas_world_frame(1:3,zz),1, size(cur_GM_mu,2)) - cur_GM_mu;
            dist = vecnorm(matrix_dist);
            if min(dist) >= filter.adaptive_birth_dist_thres 
                % Genrerate new birth component if the measurement is not close to any existing landmark/target
                new_birth_inten = horzcat (new_birth_inten, filter.birthGM_intensity);
                new_birth_mu = cat (2,new_birth_mu, new_meas_world_frame(1:3,zz));
                new_birth_cov = cat (3, new_birth_cov, filter.birthGM_cov);
            end
        end
    else
        new_birth_inten = ones(1, size(measurement,2)) *  filter.birthGM_intensity;
        new_birth_mu = new_meas_world_frame(1:3);
        new_birth_cov = repmat(filter.birthGM_cov,[1,1,size(measurement,2)]);
    end


    for par_ind = 1:size(particle,2)
        % Add birth terms and in a sense, performing time update
        particle(1,par_ind).gm_cov = cat(3,particle(1,par_ind).gm_cov, new_birth_cov);
        particle(1,par_ind).gm_inten = horzcat(particle(1,par_ind).gm_inten, new_birth_inten);
        particle(1,par_ind).gm_mu = horzcat(particle(1,par_ind).gm_mu, new_birth_mu);

        n_new_birth = size(new_birth_inten,2);
        if n_new_birth > 0
            card_pred = zeros(1,filter.max_card+1);
    
            % Convolve survival card_dist and birth_dist
            for nn = 0:filter.max_card
                ind_n = nn + 1;
                terms = zeros(filter.max_card+1,1);
                for jj = 0:nn
                    ind_j = jj + 1;
                    terms(ind_j) = exp( -n_new_birth + (nn-jj) * ...
                        log(n_new_birth) - sum(log(1:nn-jj))) * particle(1,par_ind).card_dist(ind_j);
                end
    
                card_pred(ind_n) = sum(terms);
            end
    
            card_pred = card_pred/sum(card_pred,2);
            particle(1, par_ind).card_dist = card_pred;
        end

    end %par_ind = 1:size(particle,2)
end