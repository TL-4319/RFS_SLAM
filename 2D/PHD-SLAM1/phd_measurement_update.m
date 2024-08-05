function [likelihood, GM_mu_update, GM_cov_update, GM_inten_update] = ...
phd_measurement_update (particle, GM_mu, GM_cov, GM_inten, meas, filter_params)

    if strcmp(filter_params.inner_filter,'ekf')
        %% EKF inner filter update
        % Copy prev GM value
        GM_mu_prev = GM_mu;
        GM_cov_prev = GM_cov;
        GM_inten_prev = GM_inten;
        num_GM = size(GM_inten_prev,2);

        % Pre compute measurement matrices
        %[pred_z, K, S, P, Sinv] = pre_compute_update_terms_cartesian_2D(particle, ...
        %    GM_mu, GM_cov, filter_params.sensor);
        
        % Update GM components as misdetected
        GM_inten = (1 - filter_params.sensor.detect_prob) * GM_inten_prev;

        % Update GM components as detected
        meas = meas(1:2,:); % Only get the 2D measurement
        
        [qz_temp, m_temp, P_temp] = ekf_update_multiple(meas, particle, GM_mu, GM_cov, filter_params.sensor);
        
        likelipz = zeros(1,size(meas,2));
        for zz = 1:size(meas,2)
            w_temp  = filter_params.sensor.detect_prob * GM_inten_prev(:) .* ...
                qz_temp (:,zz);
            w_temp = w_temp ./ (filter_params.sensor.clutter_density + sum(w_temp));
            GM_inten = cat(2,GM_inten, w_temp');
            GM_mu = horzcat(GM_mu, m_temp(:,:,zz));
            GM_cov = cat(3,GM_cov, P_temp);
            likelipz(1,zz) = filter_params.sensor.clutter_density + sum(w_temp',2);
        end

        % likelipz = zeros(1,size(meas,2));
        % for zz = 1:size(meas,2)
        %     tau = zeros(1,num_GM);
        %     likelipf = tau;
        %     for jj = 1:num_GM
        %         tau(1,jj) =  filter_params.sensor.detect_prob * GM_inten_prev(jj) * ...
        %                 mvnpdf(meas(:,zz),pred_z(:,jj),S(:,:,jj));
        %         if GM_inten_prev(jj) > filter_params.GM_inten_thres
        %             likelipf(:,jj) =  tau(1,jj); % Only include strong GM in particle likilihood calculation
        %         end
        %         mu = GM_mu_prev(:,jj) + K(:,:,jj) * (meas(:,zz) - pred_z(:,jj));
        %         GM_mu = horzcat(GM_mu, mu);
        %         GM_cov = cat(3,GM_cov, P(:,:,jj));
        %     end %jj = 1:num_GM
        %     likelipz(1,zz) = filter_params.sensor.clutter_density + sum(likelipf,2);
        %     sum_tau = filter_params.sensor.clutter_density + sum(tau,2);
        % 
        %     for jj = 1:num_GM
        %         nu = tau(1,jj)/sum_tau;
        %         GM_inten = horzcat(GM_inten, nu);
        %     end
        % end %zz = 1:size(meas,2)
        
        %% Particle likelihood calc
        if strcmp(filter_params.likelihood_method,'single-cluster')
            likelihood = exp(sum(GM_inten_prev,2)) * ...
                (prod(likelipz,2) + 1e-99) * particle.w;
        else
            error_msg = strcat(filter_params.likelihood_method, " likelihood is not supported");
            error(error_msg);
        end

        %% Output
        GM_mu_update = GM_mu;
        GM_cov_update = GM_cov;
        GM_inten_update = GM_inten;


    else
        error_msg = strcat(filter_params.inner_filter, " inner filter is not supported");
        error(error_msg);
    end %inner filter selection
end
