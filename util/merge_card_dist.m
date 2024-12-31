function card_dist_combined = merge_card_dist (card_dist1, card_dist2, filter)
    card_dist_combined = zeros(1, filter.max_card+1);

    card_dist1 = horzcat(card_dist1, zeros(1,filter.max_card - filter.cluster_max_card));

    card_dist2 = horzcat(card_dist2, zeros(1,filter.max_card - filter.cluster_max_card));

    % Convolution of card_dist1 and card_dist2
    for nn = 0:filter.max_card
        ind_n = nn + 1;
        terms = zeros(filter.max_card + 1, 1);
        for jj = 0:nn
            ind_j = jj + 1;
            terms(ind_j) = card_dist1(nn - jj + 1) * card_dist2(ind_j);
        end
        card_dist_combined(ind_n) = sum(terms);

    end
    
    card_dist_combined = card_dist_combined / sum(card_dist_combined,2);
end