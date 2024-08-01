function plot_3D_phd (map_est, sigma_mult, inten_cutoff, transparency)
    % Plot the 3D PHD as ellipsoids
    % Ellipsoid has center at the GM component mu
    %
    % The size of the ellipsoid corresponds to the covariance times a
    % multiplier for ease of visualizing
    % 
    % The color of ellipsoid corresponds to the intensity of the component
    
    hold on
    for ii = 1:size(map_est.max_likeli_gm_inten,2)
        inten = map_est.max_likeli_gm_inten(1,ii)/2;
        if inten < inten_cutoff
            continue
        end

        h = plot_gaussian_ellipsoid(map_est.max_likeli_gm_mu(:,ii),...
            map_est.max_likeli_gm_cov(:,:,ii)*sigma_mult, inten);
        set(h,'facealpha',transparency,'EdgeColor','none','FaceColor','flat')
        clim([inten_cutoff,1.1])
    end
end