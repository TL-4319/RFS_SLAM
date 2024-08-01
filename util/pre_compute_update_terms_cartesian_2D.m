function [pred_z, K, S, P, Sinv] = pre_compute_update_terms_cartesian_2D...
    (particle, GM_mu, GM_cov, sensor_params)
    num_GM = size(GM_mu,2);
    
    temp_mu = vertcat(GM_mu, zeros(1,num_GM));

    [~, pred_z, ~] = gen_meas_cartesian_2D(particle.pos, particle.quat, temp_mu, sensor_params); 
    
    pred_z = pred_z(1:2,:);
    K = zeros(2,2,num_GM);
    P = K;
    S = K;
    Sinv = K;
    H_3d = quat2rot(compact(particle.quat),"frame");
    H = H_3d(1:2,1:2); %Only need the 2D rotation component of the rot matrix
    for jj = 1:num_GM
        S(:,:,jj) = H * GM_cov(:,:,jj) * H' + sensor_params.R;
        S(:,:,jj) = (S(:,:,jj) + S(:,:,jj)')/2; % Avoid numerical instability
        Sinv(:,:,jj) = pinv(S(:,:,jj));
        K(:,:,jj) = GM_cov(:,:,jj) * H' * Sinv(:,:,jj);

        % Cov update via Joeseph form
        temp = (eye(2) - K(:,:,jj) * H);
        P(:,:,jj) = temp * GM_cov(:,:,jj) * temp' + K(:,:,jj) * sensor_params.R * K(:,:,jj)';
    end

end