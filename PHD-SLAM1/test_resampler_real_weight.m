clear 
close all
clc

addpath ../util

load weight_error.mat; 


filter_weight_after_norm = wei_ud;

num_particle = size(filter_weight_after_norm,2);
filter_particle_after = 1:num_particle;


%% Test resample
tic
%resample_id = particle_resample(filter_weight_after_norm,num_particle); % Lin Gao
resample_id = low_variance_resample(filter_weight_after_norm,num_particle); % Low variance resample 
toc
state_after_resample = filter_particle_after(1,resample_id);
%weight_after_resample = filter_weight_after_norm(1,resample_id);
%weight_after_resample = ones(1,num_particle) * 1/num_particle;

% figure()
% plot (pdf_query, pdf_normal,'LineWidth',2)
% hold on
% plot (pdf_query, post_pdf,'LineWidth',2);
% xlim([-50 100])

figure(1)
scatter(filter_particle_after, filter_weight_after_norm)
%ylim([0,0.02])
hold on
%scatter(filter_particle_after, filter_weight_after_norm)
prev_state = 0;
k = 0;
for i = 1:num_particle
    if state_after_resample(i) ~= prev_state
        scatter(state_after_resample(i),0.03,'k.')
        k = 0;
    else 
        k = k + 0.0015;
        scatter(state_after_resample(i), 0.03 + k,'k.')
    end
    prev_state = state_after_resample(i);
end
hold off



function new_state = propagate_state (state_prev, noise)
    if noise == 1
        new_state = state_prev.^1 + 2 * state_prev + cos(state_prev) + rand(1,size(state_prev,2)) * 0.1;
    else
        new_state = state_prev.^1 + 2 * state_prev + cos(state_prev);
    end
end