function card_dist = calc_card_dist_MB (GM_inten, cluster_max_card)
    % Construct the cardinality distribution using method proposed by Yohan
    % Petetin which treat GM components as MB 
    % 
   
    % Limit range of inten from 0 to approx 1 as defined by Mahler and for numerical eval
    GM_inten(GM_inten > 0.9) = 0.9;

    num_GM = size(GM_inten,2);
    
    % Pre compute 1 - w_i
    inv_inten = ones(1, num_GM) - GM_inten;

    % Product over all non-existent 
    prod_inv_inten = prod(inv_inten);

    card_dist = zeros(1,cluster_max_card+1);
    card_dist(1) = prod_inv_inten; % Prob (N = 0) by definition

    XI_vals = zeros(1,num_GM); % Input to the esf
    for nn = 1:num_GM
        XI_vals(1,nn) = GM_inten(nn) / (inv_inten(nn));
    end
    
    esfvals = esf(XI_vals);

    for card_val = 1:min(num_GM,cluster_max_card)
        ind_n = card_val + 1;
        card_dist(ind_n) = prod_inv_inten * esfvals(card_val);
    end

    card_dist = card_dist / sum(card_dist,2);
    
end