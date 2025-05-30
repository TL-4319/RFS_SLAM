function particle = adaptive_birth_and_time_update_CPHD_3D (measurement,...
    filter, prev_particle)
%% Generate GM component from measurements if far enough from a previously mapped GM component
particle = prev_particle; % Pre-allocating

for par_ind = 1:size(particle,2)
    new_birth_mu = []; new_birth_inten = []; new_birth_cov = [];

    % Calc sensor pos
    [sensor_pos, sensor_quat] = get_sensor_pose(particle(1,par_ind).pos,...
        particle(1,par_ind).quat, filter.sensor);

    % Reproject meas back to global frame
    new_meas_world_frame = reproject_meas(sensor_pos, sensor_quat, measurement,filter.sensor);

    cur_GM_mu = particle(1,par_ind).gm_mu;

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

    %n_new_birth = size(new_birth_inten,2);
    n_new_birth = sum(new_birth_inten,2);

    % survival cardinality
    survive_card_pred = zeros(1,filter.max_card+1);

    for nn = 0:filter.max_card
        ind_n = nn + 1;
        terms = zeros(filter.max_card+1,1);
        for jj = nn:filter.max_card
            ind_j = jj + 1;
            terms(ind_j) = exp(sum(log(1:jj))-sum(log(1:nn))-...
                sum(log(1:jj-nn)) + nn * log(filter.survive_prob)+...
                (jj-nn)*log(1 - filter.survive_prob)) * particle(1,par_ind).card_dist(ind_j);
        end
        survive_card_pred(ind_n) = sum(terms);
    end
    

    % GM component do not move but inflate uncertainty
    for jj = 1:size(cur_GM_mu,2)
        particle(1,par_ind).gm_cov(:,:,jj) = ...
            particle(1,par_ind).gm_cov(:,:,jj) + filter.map_Q;
    end

    if n_new_birth > 0
        % Add birth terms
        particle(1,par_ind).gm_cov = cat(3,particle(1,par_ind).gm_cov, new_birth_cov);
        particle(1,par_ind).gm_inten = horzcat(particle(1,par_ind).gm_inten, new_birth_inten);
        particle(1,par_ind).gm_mu = horzcat(particle(1,par_ind).gm_mu, new_birth_mu);

        card_pred = zeros(1,filter.max_card+1);

        % Convolve survival card_dist and birth_dist. Birth cardinality
        % is Poisson distributed parameterized by n_new_birth
        for nn = 0:filter.max_card
            ind_n = nn + 1;
            terms = zeros(filter.max_card+1,1);
            for jj = 0:nn
                ind_j = jj + 1;
                terms(ind_j) = exp( -n_new_birth + (nn-jj) * ...
                    log(n_new_birth) - sum(log(1:nn-jj))) * survive_card_pred(ind_j);
            end

            card_pred(ind_n) = sum(terms);
        end

        card_pred = card_pred/sum(card_pred,2);
        particle(1, par_ind).card_dist = card_pred;
    else 
        particle(1, par_ind).card_dist = survive_card_pred / sum(survive_card_pred,2);
    end

end %par_ind = 1:size(particle,2)
end