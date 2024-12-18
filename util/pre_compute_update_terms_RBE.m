function [pred_z, K, S, P, Sinv] = pre_compute_update_terms_RBE...
    (particle, GM_mu, GM_cov, sensor_params)
    num_GM = size(GM_mu,2);
    
    temp_mu = GM_mu;
    
    % Sensor pos
    [sensor_pos, sensor_quat] = get_sensor_pose(particle.pos, particle.quat, sensor_params);
    
    pred_z = zeros(3,num_GM);
    [pred_z(1,:), pred_z(2,:), pred_z(3,:)] = calc_rbe_in_body(GM_mu, sensor_pos, sensor_quat);
    
    K = zeros(3,3,num_GM);
    P = K;
    S = K;
    Sinv = K;

    H = compute_H_rbe(sensor_pos, sensor_quat, temp_mu);
    
    

    for jj = 1:num_GM
        S(:,:,jj) = H(:,:,jj) * GM_cov(:,:,jj) * H(:,:,jj)' + sensor_params.R;
        S(:,:,jj) = (S(:,:,jj) + S(:,:,jj)')/2; % Avoid numerical instability
        Sinv(:,:,jj) = pinv(S(:,:,jj));
        K(:,:,jj) = GM_cov(:,:,jj) * H(:,:,jj)' * Sinv(:,:,jj);

        % Cov update via Joeseph form
        temp = (eye(3) - K(:,:,jj) * H(:,:,jj));
        P(:,:,jj) = temp * GM_cov(:,:,jj) * temp' + K(:,:,jj) * sensor_params.R * K(:,:,jj)';
    end

end