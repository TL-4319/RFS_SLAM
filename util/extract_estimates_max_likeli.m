function [pose_est, map_est] = extract_estimates_max_likeli (particle, filter_params)
    %% Extract the pose and map estimate using maximum likelihood
    likeli_vec = zeros(1,size(particle,2));
    for par_ind = 1:size(particle,2)
        likeli_vec(par_ind) = particle(1,par_ind).w;
    end
    [~, max_w_particle_ind] = max(likeli_vec);

    % Trajectory estimate
    pose_est.pos = particle(1,max_w_particle_ind).pos;
    pose_est.quat = particle(1,max_w_particle_ind).quat;

    % MAP Landmark estimation
    max_likeli_gm_mu = particle(1,max_w_particle_ind).gm_mu;
    max_likeli_gm_inten = particle(1,max_w_particle_ind).gm_inten;
    max_likeli_gm_cov = particle(1,max_w_particle_ind).gm_cov; % Used for visualization
    
    
    % Find expected number of landmark
    if strcmp(filter_params.map_est_method,'exp')
        map_est.exp_num_landmark = round(sum(max_likeli_gm_inten));
        [~,ID_map] = maxk (max_likeli_gm_inten, map_est.exp_num_landmark);
    elseif strcmp(filter_params.map_est_method,'thres')
        [~, ID_map] = find(max_likeli_gm_inten > filter_params.GM_inten_thres);
        map_est.exp_num_landmark = size(any(ID_map),1);
    elseif strcmp(filter_params.map_est_method,'cphd')
        max_likeli_card_dist = particle(1,max_w_particle_ind).card_dist;
        [~,max_card_ind] = max(max_likeli_card_dist);
        map_est.exp_num_landmark = min([max_card_ind - 1,size(max_likeli_gm_mu,2)]);
        [~,ID_map] = maxk (max_likeli_gm_inten, map_est.exp_num_landmark);
    end
    map_est.feature_pos = max_likeli_gm_mu(:,ID_map);
    map_est.max_likeli_gm_mu = max_likeli_gm_mu;
    map_est.max_likeli_gm_cov = max_likeli_gm_cov;
    map_est.max_likeli_gm_inten = max_likeli_gm_inten;
    
end