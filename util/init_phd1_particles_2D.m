function p = init_phd1_particles_2D(num_particles, init_pos, init_quat, ...
    meas_inworld, def_cov, def_inten)
    % Position of the particle is 3D form but propagate only 2D motion
    % Map is represented with 2D representation for consistency
    for i = 1:num_particles
        p(i).w = 1/num_particles;
        p(i).pos = init_pos;
        p(i).quat = init_quat;
        
        p(i).gm_mu = meas_inworld(1:2,:);
        p(i).gm_cov = repmat(def_cov,1,1,size(meas_inworld,2));
        p(i).gm_inten = ones(1,size(meas_inworld,2)) * def_inten;
    end
end