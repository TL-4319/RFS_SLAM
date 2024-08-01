function [particle, num_effective_particle] = resample_particles (prev_particle, filter)
    %% Apply stratified resample
    particle = prev_particle; % Pre-allocate

    % Normalize weight
    likeli_vec = zeros(1,size(particle,2));
    for par_ind = 1:size(particle,2)
        likeli_vec(par_ind) = particle(1,par_ind).w;
    end
    normalize_likeli_vec = likeli_vec / sum(likeli_vec,2);
    num_effective_particle = 1/(sum(normalize_likeli_vec.^2,2));

    if num_effective_particle < filter.resample_threshold * size(particle,2)
        % Print out message if resample
        dbg_str = sprintf('Num of effective particle is %d . Resample triggered', num_effective_particle);
        disp (dbg_str)
        
        % Resmaple and generate new particles
        resample_ind = low_variance_resample(normalize_likeli_vec, size(particle,2));
        for par_ind = 1:filter.num_particle
            particle(1,par_ind).pos = prev_particle(1,resample_ind(1,par_ind)).pos;
            particle(1,par_ind).quat = prev_particle(1,resample_ind(1,par_ind)).quat;
            particle(1,par_ind).gm_mu = prev_particle(1,resample_ind(1,par_ind)).gm_mu;
            particle(1,par_ind).gm_inten = prev_particle(1,resample_ind(1,par_ind)).gm_inten;
            particle(1,par_ind).gm_cov = prev_particle(1,resample_ind(1,par_ind)).gm_cov;
            particle(1,par_ind).w = 1/size(particle,2);
        end

    else % Normalize the particle weights if no resample is needed
        for par_ind = 1:size(particle,2)
            particle(1,par_ind).w = normalize_likeli_vec(par_ind);
        end
    end

end