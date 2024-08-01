function plot_2D_phd (map_est, sigma_mult, inten_cutoff, transparency)
    % Plot the 2D PHD as ellipsoids
    % This function append the 2D GM component with appropriate 3D dummy
    % variables and just call the 3D phd plot function
    num_GM = size(map_est.max_likeli_gm_mu,2);

    temp_cov = zeros(3,3,num_GM);

    map_est.max_likeli_gm_mu = vertcat(map_est.max_likeli_gm_mu,2*ones(1, num_GM));
    for jj = 1:num_GM
        cov_vec = vertcat(diag(map_est.max_likeli_gm_cov(:,:,jj)),0.0000001);
        temp_cov(:,:,jj) = diag(cov_vec);
    end
    map_est.max_likeli_gm_cov = temp_cov;
    
    plot_3D_phd(map_est, sigma_mult, inten_cutoff, transparency);
end