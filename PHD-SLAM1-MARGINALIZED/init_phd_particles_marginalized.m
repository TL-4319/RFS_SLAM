function p = init_phd_particles_marginalized(num_particles, init_pos, init_vel, ...
    init_quat, init_gyro_bias, init_acc_bias, init_bias_cov,meas_inworld, def_cov, def_inten)
    for i = 1:num_particles
        % Particle weight
        p(i).w = 1/num_particles;
        
        % Non-linear components
        p(i).pos = init_pos;
        p(i).vel = init_vel;
        p(i).quat = init_quat;
        
        % Linear components
        p(i).gyro_bias = init_gyro_bias;
        p(i).acc_bias = init_acc_bias;
        p(i).bias_cov = init_bias_cov;

        % Map gaussian mixture components
        p(i).gm_mu = meas_inworld;
        p(i).gm_cov = repmat(def_cov,1,1,size(meas_inworld,2));
        p(i).gm_inten = ones(1,size(meas_inworld,2)) * def_inten;


    end
end