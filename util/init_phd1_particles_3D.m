function p = init_phd1_particles_3D(num_particles, init_pos, init_quat, ...
    meas_inworld, def_cov, def_inten)

    for i = 1:num_particles
        p(i).w = 1/num_particles;
        p(i).pos = init_pos;
        p(i).quat = init_quat;
        
        p(i).gm_mu = meas_inworld(1:3,:);
        p(i).gm_cov = repmat(def_cov,1,1,size(meas_inworld,2));
        p(i).gm_inten = ones(1,size(meas_inworld,2)) * def_inten;
    end
end