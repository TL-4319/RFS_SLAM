function qz = calc_meas_likelihood (meas, pred_meas, S, invS)
    num_GM = size(pred_meas,2);
    num_meas = size(meas,2);

    qz = zeros(num_GM, num_meas);

    for jj = 1:num_GM
        S_jj = S(:,:,jj);
        invS_jj = invS(:,:,jj);
        Vs = chol(S_jj);
        det_S = prod(diag(Vs))^2;
        eta = pred_meas(:,jj);

        qz(jj,:) = exp(-0.5*size(meas,1)*log(2*pi) - 0.5*log(det_S) -...
            0.5*dot(meas-repmat(eta,[1 num_meas]),...
            invS_jj*(meas-repmat(eta,[1 num_meas]))))';
    end
end