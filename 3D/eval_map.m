close all
clear
clc

addpath('../util/')

ospa_c = 1;
ospa_p = 2;
cola_c = 1;
cola_p = 2;

%% Select simulation
load('sim_result/sim-20250731-0738-pd04.mat');


%% 
time_vec = simulation.truth.time_vec;
dt = time_vec(2) - time_vec(1);
sensor_time_ind = 1:5:size(simulation.truth.sensor_time_vec,2);
sensor_time_vec = simulation.truth.sensor_time_vec(sensor_time_ind);

% Preallocate mapping error metrics
true_card = zeros(1, size(simulation.truth.cummulative_landmark_in_FOV,1));
for ii = 1:size(simulation.truth.cummulative_landmark_in_FOV,1)
    true_card(ii) = size(simulation.truth.cummulative_landmark_in_FOV{ii,1},2);
end

% Cardinality
est_card = zeros(size(simulation.result,1), size(sensor_time_vec,2));

% Time varying OSPA
ospa_1 = est_card;
ospa_2 = est_card;
ospa_3 = est_card;

% Total COLA metric
cola_1 = est_card;
cola_2 = est_card;
cola_3 = est_card;

test = get_comps(simulation.truth.cummulative_landmark_in_FOV{end,1},[1,2,3]);

for ii = 1:size(simulation.result,1)
    for kk = 2:size(sensor_time_vec,2)
        est_card(ii,kk) = simulation.result{ii,1}.filter_est.map_est{kk,1}.exp_num_landmark;

        % % Mapping error
        % if ii == 1 
        %     % This only needs to be done once
        %     true_map = get_comps(simulation.truth.cummulative_landmark_in_FOV{end,1},[1,2,3]);
        % end
        
        true_map = get_comps(simulation.truth.cummulative_landmark_in_FOV{end,1},[1,2,3]);

        est_map = get_comps(simulation.result{ii,1}.filter_est.map_est{sensor_time_ind(kk),1}.feature_pos,[1,2,3]);

        [ospa_1(ii,kk), ospa_2(ii,kk), ospa_3(ii,kk)] = ospa_dist (true_map,...
        est_map, ospa_c, ospa_p);

        % [cola_1(ii,kk), cola_2(ii,kk), cola_3(ii,kk)] = cola_dist (true_map,...
        % est_map, cola_c, cola_p);
    end
end
%%
mean(ospa_1(:,end),1)
std(ospa_1(:,end),1)

figure()
plot(mean(ospa_1(:,2:end),1))
ospa = vertcat(sensor_time_ind(2:end), mean(ospa_1(:,2:end),1), ...
    mean(ospa_2(:,2:end),1), mean(ospa_3(:,2:end),1));

%% Util functions
function Xc= get_comps(X,c)
    if isempty(X)
        Xc= [];
    else
        Xc= X(c,:);
    end
end