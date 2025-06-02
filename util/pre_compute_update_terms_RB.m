function [pred_z, K, S, P, Sinv] = pre_compute_update_terms_RB...
    (particle, GM_mu, GM_cov, sensor_params)
    num_GM = size(GM_mu,2);
    
    pred_z = zeros(2,num_GM);
    z_padded_GM_mu = vertcat(GM_mu, zeros(1,size(GM_mu,2))); % Pad z for GM mu with 0 to reuse math functions
    [pred_z(1,:), pred_z(2,:), ~] = calc_rbe_in_body(z_padded_GM_mu, particle.pos, particle.quat);
    
    K = zeros(2,2,num_GM);
    P = K;
    S = K;
    Sinv = K;

    H = compute_H_rb(particle.pos, particle.quat, z_padded_GM_mu);

    for jj = 1:num_GM
        S(:,:,jj) = H(:,:,jj) * GM_cov(:,:,jj) * H(:,:,jj)' + sensor_params.R;
        %S(:,:,jj) = (S(:,:,jj) + S(:,:,jj)')/2; % Avoid numerical instability
        Sinv(:,:,jj) = pinv(S(:,:,jj));
        K(:,:,jj) = GM_cov(:,:,jj) * H(:,:,jj)' * Sinv(:,:,jj);

        % Cov update via Joeseph form
        temp = (eye(2) - K(:,:,jj) * H(:,:,jj));
        P(:,:,jj) = temp * GM_cov(:,:,jj) * temp' + K(:,:,jj) * sensor_params.R * K(:,:,jj)';
    end

end