function particle_likelihood = single_cluster_likelihood(meas, pred_meas, GM_inten, Sinv, S, Pd, clutter)
    num_meas = size(meas,2);
    num_gm_in = size(pred_meas,2);
    meas_dim = size(meas,1);

    likeli_meas = zeros(1, num_meas);
    for zz = 1:num_meas
        likeli_gm = zeros(1, num_gm_in);
        for jj = 1:num_gm_in
            zdiff = meas(:,zz) - pred_meas(:,jj);
            likeli_gm(:,jj) = 1 / sqrt(det(S(:,:,jj)) * (2 * pi)^meas_dim) * ...
                exp(-0.5*(zdiff)' * Sinv(:,:,jj) * zdiff) ...
                * GM_inten(jj);
             
        end
        likeli_meas(:,zz) = clutter + sum(likeli_gm * Pd,2);
    end
    particle_likelihood = exp(sum(GM_inten,2)) * (prod(likeli_meas,2)  + 1e-99);
end