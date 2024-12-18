function [likelihood, GM_mu_update, GM_cov_update, GM_inten_update, card_dist] = ...
cphd_measurement_update (particle, GM_mu, GM_cov, GM_inten, card_dist, ...
detect_prob_vec, meas, filter_params)
    
    % Utilize code from Vo's implementation of CPHD
    
    num_meas = size(meas,2);
    if strcmp(filter_params.inner_filter,'ekf')
        %% EKF inner filter update
        % Copy prev GM value
        GM_mu_prev = GM_mu;
        GM_cov_prev = GM_cov;
        GM_inten_prev = GM_inten;
        num_GM = size(GM_inten_prev,2);

        % Pre compute measurement matrices
        [pred_z, K, S, P, Sinv] = pre_compute_update_terms_RBE(particle, ...
            GM_mu, GM_cov, filter_params.sensor);

        % Pre compute measurement likelihood of meas zz to gm component jj
        meas_likelihood = calc_meas_likelihood(meas, pred_z, S, Sinv);

        % Pre compute for elementary symmetric function
        XI_vals = zeros(1,num_meas); % input to esf
        for zz = 1:num_meas
            XI_vals(zz) = (detect_prob_vec .* GM_inten) * meas_likelihood(:,zz) ...
                / filter_params.sensor.clutter_density;
        end
        
        esfvals_E = esf(XI_vals); %calculate esf for entire observation set
        esfvals_D = zeros(num_meas, num_meas); %calculate esf with each observation index removed one-by-one
        
        for zz = 1:num_meas
            esfvals_D(:,zz) = esf([XI_vals(1:zz-1),XI_vals(zz+1:num_meas)]);
        end

        % Pre calc for upsilons
        upsilon0_E = zeros(filter_params.max_card+1, 1);
        upsilon1_E = upsilon0_E;
        upsilon1_D = zeros(filter_params.max_card+1, num_meas);

        mis_detect_prob_inner_prod_est = (ones(size(detect_prob_vec)) - detect_prob_vec) *...
            GM_inten'; % Approx of <1-P_d, inten> by James McCabe

        for nn = 0:filter_params.max_card
            ind_n = nn + 1;

            term0_E = zeros(min(num_meas, nn)+1, 1); 

            term1_E = term0_E;

            for jj = 0:min(num_meas, nn)
                ind_j = jj + 1;
                
                %calculate upsilon0_E(idxn)
                term0_E(ind_j) = exp( -filter_params.sensor.avg_num_clutter + (num_meas - jj) * log(filter_params.sensor.avg_num_clutter)+...
                    sum(log(1:nn)) - sum(log(1:nn-jj)) +...
                    (nn - jj) * log(mis_detect_prob_inner_prod_est) -...
                    jj * log(sum(GM_inten))) * esfvals_E(ind_j);
                
                %calculate upsilon1_E(idxn)
                if nn >= jj+1
                    term1_E(ind_j) = exp( -filter_params.sensor.avg_num_clutter + (num_meas - jj) * log(filter_params.sensor.avg_num_clutter)+...
                    sum(log(1:nn)) - sum(log(1:nn-(jj+1))) +...
                    (nn - (jj+1)) * log(mis_detect_prob_inner_prod_est) -...
                    (jj+1) * log(sum(GM_inten))) * esfvals_E(ind_j);
                end

                

            end % jj = 0:min(num_meas, nn)

            upsilon0_E(ind_n) = sum(term0_E);
            upsilon1_E(ind_n) = sum(term1_E);
            
            %calculate upsilon1_D(idxn,:) if m>0
            term1_D = zeros(min(num_meas-1,nn)+1, num_meas);

            for zz = 1:num_meas
                for jj = 0:min(num_meas-1,nn)
                    ind_j = jj + 1;
                    if nn > jj + 1
                        term1_D(ind_j, zz) = exp( -filter_params.sensor.avg_num_clutter + ((num_meas-1) - jj) * log(filter_params.sensor.avg_num_clutter)+...
                    sum(log(1:nn)) - sum(log(1:nn-(jj+1))) +...
                    (nn - (jj+1)) * log(mis_detect_prob_inner_prod_est) -...
                    (jj+1) * log(sum(GM_inten))) * esfvals_D(ind_j,zz);
                    end
                end
            end

            upsilon1_D(ind_n,:) = sum(term1_D,1);

        end %nn = 0:filter_params.max_card

        %% GM update step
        % Update GM components as misdetected
        GM_inten = (upsilon1_E' * card_dist')/(upsilon0_E' * card_dist') *...
            (1 - filter_params.sensor.detect_prob) * GM_inten_prev;

        % Update GM components as detected
        likelipz = zeros(1,size(meas,2));
        for zz = 1:size(meas,2)
            tau = zeros(1,num_GM);
            likelipf = tau;
            for jj = 1:num_GM
                tau(1,jj) =  detect_prob_vec(jj) * GM_inten_prev(jj) * ...
                        meas_likelihood(jj,zz);             %mvnpdf(meas(:,zz),pred_z(:,jj),S(:,:,jj));
                if GM_inten_prev(jj) > filter_params.GM_inten_thres
                    likelipf(:,jj) =  tau(1,jj); % Only include strong GM in particle likilihood calculation
                end
                mu = GM_mu_prev(:,jj) + K(:,:,jj) * (meas(:,zz) - pred_z(:,jj));
                GM_mu = horzcat(GM_mu, mu);
                GM_cov = cat(3,GM_cov, P(:,:,jj));

                nu = (upsilon1_D(:,zz)' * card_dist')/(upsilon0_E' * card_dist') * ...
                detect_prob_vec(jj) .* meas_likelihood(jj,zz)/filter_params.sensor.clutter_density .* GM_inten_prev(jj);
                
                GM_inten = horzcat(GM_inten, nu);
            end %jj = 1:num_GM
            likelipz(1,zz) = filter_params.sensor.clutter_density + sum(likelipf,2);
            sum_tau = filter_params.sensor.clutter_density + sum(tau,2);
                 

        end %zz = 1:size(meas,2)
        
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
        % Card update
        card_dist = upsilon0_E' .* card_dist;
        card_dist = card_dist/sum(card_dist,2); % Normalize

    else
        error_msg = strcat(filter_params.inner_filter, " inner filter is not supported");
        error(error_msg);
    end %inner filter selection
end
