function [pose_est, map_est] = extract_estimates_max_likeli (particle)
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
    map_est.exp_num_landmark = round(sum(max_likeli_gm_inten));
    [~,ID_map] = maxk (max_likeli_gm_inten, map_est.exp_num_landmark);
    map_est.feature_pos = max_likeli_gm_mu(:,ID_map);
    map_est.max_likeli_gm_mu = max_likeli_gm_mu;
    map_est.max_likeli_gm_cov = max_likeli_gm_cov;
    map_est.max_likeli_gm_inten = max_likeli_gm_inten;
    
end