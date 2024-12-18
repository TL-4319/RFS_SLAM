function p = init_cphd1_particles_3D(init_pos, init_quat, ...
    meas_inworld, filter_params)
    
    % Construct cardinality distribution based on reprojected measurements
    % i.e. probability of 1 at the number of birth

    card_dist = zeros(1,filter_params.max_card+1);
    card_dist(size(meas_inworld,2)+1) = 1;
    
    for i = 1:filter_params.num_particle
        p(i).w = 1/filter_params.num_particle;
        p(i).pos = init_pos;
        p(i).quat = init_quat;
        
        p(i).card_dist = card_dist;
        p(i).gm_mu = meas_inworld(1:3,:);
        p(i).gm_cov = repmat(filter_params.birthGM_cov,1,1,size(meas_inworld,2));
        p(i).gm_inten = ones(1,size(meas_inworld,2)) * filter_params.birthGM_intensity;
    end
end