close all
clear 
clc

cola_c = 0.5;
cola_p = 2;

%% Define eval run
file_name = ["mapping_seed69_min20cm", "mapping_seed69_min50cm", "mapping_seed69_min100cm",...
    "mapping_seed69_min200cm", "mapping_seed69_min300cm", "mapping_seed69_min500cm"];

disp_name = ["0.2m","0.5m", "1.0m","2.0m", "3.0m","5.0m"];

%%
% Each cell is a eval run 
% Within each cell
% cola_error 
%       [num_monte_carla x num_epoch] array of COLA distance
% card_truth
%       [1 x num_epoch] array of true map cardinality
% card_est
%       [num_monte_carla x num_epoch] array of estimated map cardinality

cola_error = cell(size(file_name,2),1);
card_truth = cola_error;
card_est = cola_error;

for ii = 1:size(file_name,2)
    sim_data = load(strcat("sim_result\",file_name(ii),".mat"));
    
    truth_map = sim_data.simulation.truth.cummulative_landmark_in_FOV;
    
    num_run = size(sim_data.simulation.result,1);

    num_epoch = size(sim_data.simulation.result{1}.meas_table,1);

    cur_card = zeros(num_run,num_epoch);
    cur_cola_error = zeros(num_run, num_epoch);
    cur_card_truth = zeros(1,num_epoch);

    for jj = 1:num_run
        for kk = 2:num_epoch
            if jj == 1
                cur_card_truth(1,kk) = size(truth_map{kk},2);
                true_map = get_comps(truth_map{kk},[1,2]);
            end 
            cur_card(jj,kk) = ...
                sim_data.simulation.result{jj}.filter_est.map_est{kk}.exp_num_landmark;
            
            est_map = get_comps(sim_data.simulation.result{jj}.filter_est.map_est{kk}.feature_pos,[1,2]);
            cur_cola_error(jj,kk) = cola_dist (true_map,...
            est_map, cola_c, cola_p);
        end %for kk = 2:num_epoch
    end %for jj = 1:num_run
    
    card_truth{ii} = cur_card_truth;
    card_est{ii} = cur_card;
    cola_error{ii} = cur_cola_error;
end %for ii = 1:size(file_name,2)

%% Plotting
for ii = 1:size(file_name,2)
    if ii == 1
        figure(1)
        hold off

        cur_cola_error = cola_error{ii};
        cur_cola_mean = mean(cur_cola_error, 1);
        cur_cola_std = std(cur_cola_mean,0,1);
        plot(cur_cola_mean, "DisplayName",disp_name(ii));
        hold on
    else
        cur_cola_error = cola_error{ii};
        cur_cola_mean = mean(cur_cola_error, 1);
        cur_cola_std = std(cur_cola_mean,0,1);
        plot(cur_cola_mean, "DisplayName",disp_name(ii));
    end
end %for ii = 1:size(file_name,2)
legend

%% Visualize of map
fig = figure(2);
title ("Sim world")
fig.Position = [1,1,1000,1000];


%% Util functions
function Xc= get_comps(X,c)
    if isempty(X)
        Xc= [];
    else
        Xc= X(c,:);
    end
end