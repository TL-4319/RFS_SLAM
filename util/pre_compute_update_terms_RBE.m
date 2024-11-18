function [pred_z, K, S, P, Sinv] = pre_compute_update_terms_RBE...
    (particle, GM_mu, GM_cov, sensor_params)
    num_GM = size(GM_mu,3);
    
    temp_mu = vertcat(GM_mu, zeros(1,num_GM));

    [~, pred_z, ~] = gen_meas_RBE_3D(particle.pos, particle.quat, temp_mu, sensor_params); 
    
    pred_z = pred_z(1:3,:);
    K = zeros(3,3,num_GM);
    P = K;
    S = K;
    Sinv = K;
    H_3d = quat2rot(compact(particle.quat),"frame");

    for jj = 1:num_GM
        S(:,:,jj) = H_3d * GM_cov(:,:,jj) * H_3d' + sensor_params.R;
        S(:,:,jj) = (S(:,:,jj) + S(:,:,jj)')/2; % Avoid numerical instability
        Sinv(:,:,jj) = pinv(S(:,:,jj));
        K(:,:,jj) = GM_cov(:,:,jj) * H' * Sinv(:,:,jj);

        % Cov update via Joeseph form
        temp = (eye(3) - K(:,:,jj) * H);
        P(:,:,jj) = temp * GM_cov(:,:,jj) * temp' + K(:,:,jj) * sensor_params.R * K(:,:,jj)';
    end

end