function [GM_mu_out, GM_cov_out, GM_inten_out] = cleanup_PHD (GM_mu, ...
    GM_cov, GM_inten, pruning_thres, merge_dist, num_GM_cap)
    [mu_prune, cov_prune, inten_out] = prune_gm (GM_mu, GM_cov, GM_inten, pruning_thres);
    [mu_merge, cov_merge, inten_merge] = merge_gm_lingao (mu_prune, cov_prune, inten_out, merge_dist);
    [mu_cap, cov_cap, inten_cap] = cap_gm (mu_merge, cov_merge, inten_merge, num_GM_cap);
    GM_mu_out = mu_cap;
    GM_cov_out = cov_cap;
    GM_inten_out = inten_cap; 
end

function [GM_mu_out, GM_cov_out, GM_inten_out] = prune_gm(GM_mu, ...
    GM_cov, GM_inten, pruning_thres)
    pass_ind = find(GM_inten > pruning_thres);

    GM_mu_out = GM_mu(:,pass_ind);
    GM_cov_out = GM_cov(:,:,pass_ind);
    GM_inten_out = GM_inten(pass_ind);
end

function [GM_mu_out, GM_cov_out, GM_inten_out] = merge_gm_lingao(GM_mu, ...
    GM_cov, GM_inten, merge_dist)
    % LINGAO'S IMPLEMENTATION
    L = size(GM_inten,2);
    x_dim = size(GM_mu,1);
    I = 1:L;
    el = 1;
    
    if all(GM_inten == 0)
        % No component due to all low intensity
        GM_inten_out = [];
        GM_mu_out = [];
        GM_cov_out = [];
        return;
    end

    while ~isempty(I)
        [~, j] = max(GM_inten);
        Ij = [];
        iPt = inv(GM_cov(:,:,j));
        GM_inten_out(el,1) = 0;
        GM_mu_out (:,el) = zeros(x_dim,1); 
        GM_cov_out(:,:,el) = zeros(x_dim, x_dim);
        for i = I
            val = (GM_mu(:,i)-GM_mu(:,j))'* iPt *(GM_mu(:,i)-GM_mu(:,j));
            %val = sqrt((GM_mu(:,i)-GM_mu(:,j))'* (GM_mu(:,i)-GM_mu(:,j)));
            if val <= merge_dist
                Ij = [Ij i];
            end
        end

        GM_inten_out(el,1) = sum(GM_inten(Ij));
        GM_mu_out (:,el) = wsumvec(GM_inten(Ij), GM_mu(:,Ij), x_dim);
        GM_cov_out(:,:,el) = wsummat(GM_inten(Ij), GM_cov(:,:,Ij), x_dim);

        GM_mu_out(:,el) = GM_mu_out(:,el) / GM_inten_out(el);
        GM_cov_out(:,:,el) = GM_cov_out(:,:,el) / GM_inten_out(el);
        I = setdiff(I,Ij);
        GM_inten(Ij) = -1;
        el = el+1;
    end
    
    % Reshape vector to match
    GM_inten_out = reshape(GM_inten_out,[1,length(GM_inten_out)]);
end

function out = wsumvec(w,vecstack,xdim)
    wmat = repmat(w,[xdim,1]);
    out  = sum(wmat.*vecstack,2);
end

function out = wsummat(w,matstack,xdim)
    w = reshape(w,[1,size(w)]);
    wmat = repmat(w,[xdim,xdim,1]);
    out = sum(wmat.*matstack,3);
end

% function [GM_mu_out, GM_cov_out, GM_inten_out] = merge_gm(GM_mu, ...
%     GM_cov, GM_inten, merge_dist)
%     I = 1:size(GM_inten,2);
%     el = 0;
% 
%     w_new = []; mu_new = []; cov_new = [];
% 
%     while ~isempty(I)
%         el = el + 1;
%         [~,j] = max(GM_inten(1,I));
%         j = I(j);
%         L = [];
%         for i=I
%             % calc mahalonobis dist
%             dist = (GM_mu(:,i) - GM_mu(:,j))' * inv(GM_cov(:,:,i)) * (GM_mu(:,i) - GM_mu(:,j));
%             if dist <= merge_dist
%                 L = horzcat(L,i);
%             end
%         end
%         % New intensity
%         sum_weight = sum(GM_inten(L));        
% 
%         % New mean
%         weighted_sum = GM_inten(L) .* GM_mu(:,L);
%         weighted_sum = sum (weighted_sum,2) / sum_weight;
% 
%         %New variance
%         P_temp = zeros (3,3);
%         for ii = 1:size(L,2)
%             mean_dif = weighted_sum - GM_mu(:,L(ii));
%             cur_P = GM_inten(L(ii)) * (GM_cov(:,:,L(ii)) + mean_dif * mean_dif');
%             P_temp = P_temp + cur_P;
%         end
% 
%         w_new = horzcat(w_new, sum(GM_inten(L)));
%         mu_new = horzcat (mu_new, weighted_sum);
%         cov_new = cat (3, cov_new, P_temp);
%         I = setdiff(I,L);
%     end
% 
%     GM_mu_out = mu_new;
%     GM_cov_out = cov_new;
%     GM_inten_out = w_new;
% end


function [GM_mu_out, GM_cov_out, GM_inten_out] = cap_gm (GM_mu, ...
    GM_cov, GM_inten, num_GM_cap)
    if size(GM_inten,2) <= num_GM_cap
        GM_mu_out = GM_mu;
        GM_cov_out = GM_cov;
        GM_inten_out = GM_inten;
    else
        [~, pass_ind] = maxk (GM_inten, num_GM_cap);
        GM_mu_out = GM_mu (:,pass_ind);
        GM_cov_out = GM_cov (:,:,pass_ind);
        GM_inten_out = GM_inten (pass_ind);
    end
end