function [pred_meas, K, S, P, Sinv] = compute_update_terms (cur_particle, GM_mu, GM_cov, R)
    num_GM = size (GM_mu,2);
    pred_meas = zeros(3,num_GM);
    K = zeros (3,3,num_GM);
    P = K;
    S = K;
    Sinv = K;
    H_old = calc_meas_jacobian (cur_particle.quat);
    H = quat2rot(compact(cur_particle.quat),"frame");
    res = H_old - H
    for i = 1:num_GM
        pred_meas(:,i) = gen_pred_meas(cur_particle.pos, cur_particle.quat, GM_mu(:,i));
        S(:,:,i) = H * GM_cov(:,:,i) * H' + R;
        S(:,:,i) = (S(:,:,i) + S(:,:,i)')/2; % Avoid numerical instability
        Sinv (:,:,i) = pinv(S(:,:,i));
        K(:,:,i) = GM_cov(:,:,i) * H' * Sinv (:,:,i);
        temp = (eye(3) - K(:,:,i) * H);
        P(:,:,i) = temp * GM_cov(:,:,i) * temp' + K(:,:,i) * R * K(:,:,i)';
        %P (:,:,i) = (eye(3) - K(:,:,i) * H) * GM_cov (:,:,i);
    end
end

function H = calc_meas_jacobian (quat)
    H = zeros(3,3);
    [w,x,y,z] = parts(quat);
    H(1,1) = 0.5 - y^2 - z^2;
    H(1,2) = w * z + x * y;
    H(1,3) = x * z - w * y;
    H(2,1) = x * y - w * z;
    H(2,2) = 0.5 - x^2 - z^2;
    H(2,3) = w * x + y * z;
    H(3,1) = w * y + x * z;
    H(3,2) = y * z - w * x;
    H(3,3) = 0.5 - x^2 - y^2;
    H = H*2;
end