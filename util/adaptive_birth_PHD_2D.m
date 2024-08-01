function particle = adaptive_birth_PHD_2D (pos, quat, measurement, current_map_est, filter, prev_particle)
    %% Generate GM component from measurements if far enough from a previously mapped GM component
    particle = prev_particle; % Pre-allocating

    new_birth_mu = []; new_birth_inten = []; new_birth_cov = [];
    new_meas_world_frame = reproject_meas(pos, quat, measurement,filter.sensor);
    cur_GM_mu = current_map_est.max_likeli_gm_mu;
    for zz = 1:size(measurement,2)
        matrix_dist = repmat(new_meas_world_frame(1:2,zz),1, size(cur_GM_mu,2)) - cur_GM_mu;
        dist = vecnorm(matrix_dist);
        if min(dist) >= filter.adaptive_birth_dist_thres 
            % Genrerate new birth component if the measurement is not close to any existing landmark/target
            new_birth_inten = horzcat (new_birth_inten, filter.birthGM_intensity);
            new_birth_mu = cat (2,new_birth_mu, new_meas_world_frame(1:2,zz));
            new_birth_cov = cat (3, new_birth_cov, filter.birthGM_cov);
        end
    end


    for par_ind = 1:size(particle,2)
        particle(1,par_ind).gm_cov = cat(3,particle(1,par_ind).gm_cov, new_birth_cov);
        particle(1,par_ind).gm_inten = horzcat(particle(1,par_ind).gm_inten, new_birth_inten);
        particle(1,par_ind).gm_mu = horzcat(particle(1,par_ind).gm_mu, new_birth_mu);
    end

end