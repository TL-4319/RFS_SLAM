clear 
%close all
clc

addpath ../util

%rng(100)
num_particle = 500;
pdf_num_query = 20000;
%% Pre transition
mu = 5;
sigma = 5;
pd = makedist('Normal','mu',mu,'sigma',sigma);
pdf_query = linspace(-50, 100, pdf_num_query);
pdf_normal = pdf(pd, pdf_query);
filter_particle = normrnd(mu, sigma, 1,num_particle);
filter_weight_pre = ones(1,num_particle) * 1/num_particle;
pdf_particle = normrnd(mu,sigma,1,pdf_num_query);

%% Propagate through non-linear
edge = [-50 -50:0.1:80 100];
pdf_particle_after = propagate_state(pdf_particle,0);
post_prop_pdf = fitdist(pdf_particle_after','kernel');
post_pdf = pdf(post_prop_pdf, pdf_query);

%% Test particle fitler
filter_particle_after = propagate_state(filter_particle,1); % Simulate particle propagation
fitler_weight_after = pdf(post_prop_pdf,filter_particle_after); % Simulate importance weighing
filter_weight_after_norm = fitler_weight_after;%/ sum(fitler_weight_after,2);

%% Test resample
tic
%resample_id = particle_resample(filter_weight_after_norm,num_particle); % Lin Gao
resample_id = low_variance_resample(filter_weight_after_norm,num_particle); % Low variance resample 
toc
state_after_resample = filter_particle_after(1,resample_id);
weight_after_resample = filter_weight_after_norm(1,resample_id);
weight_after_resample = ones(1,num_particle) * 1/num_particle;

% figure()
% plot (pdf_query, pdf_normal,'LineWidth',2)
% hold on
% plot (pdf_query, post_pdf,'LineWidth',2);
% xlim([-50 100])

figure(1)
scatter(filter_particle_after, filter_weight_after_norm)
ylim([0,0.1])
hold on
%scatter(filter_particle_after, filter_weight_after_norm)
prev_state = 0;
k = 0;
for i = 1:num_particle
    if state_after_resample(i) ~= prev_state
        scatter(state_after_resample(i), weight_after_resample(i),'k.')
        k = 0;
    else 
        k = k + 0.0015;
        scatter(state_after_resample(i), weight_after_resample(i) + k,'k.')
    end
    prev_state = state_after_resample(i);
end
hold off
xlim([-30 50])


function new_state = propagate_state (state_prev, noise)
    if noise == 1
        new_state = state_prev.^1 + 2 * state_prev + cos(state_prev) + rand(1,size(state_prev,2)) * 0.1;
    else
        new_state = state_prev.^1 + 2 * state_prev + cos(state_prev);
    end
end