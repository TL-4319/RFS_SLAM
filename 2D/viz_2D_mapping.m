close all
clear
clc

addpath '../util/'

%%
file_name = "sim_result\mapping_seed69_min200cm.mat";
run_to_draw = 20;

%%
sim_data = load(file_name);

truth_map = sim_data.simulation.truth.cummulative_landmark_in_FOV;
end_ind = size(truth_map,1);
est_map = sim_data.simulation.result{run_to_draw}.filter_est.map_est{end_ind}.feature_pos;

pos = sim_data.simulation.result{run_to_draw}.filter_est.pos;
quat = sim_data.simulation.result{run_to_draw}.filter_est.quat;

fig = figure(1);
title ("Sim world")
fig.Position = [1,1,1000,1000];

% Truth map
scatter3(truth_map{end}(1,:), truth_map{end}(2,:), truth_map{end}(3,:),...
    ones(size(truth_map{end},2),1) * 50, 'k');
hold on 

% Est map
scatter3(est_map(1,:), est_map(2,:), zeros(1,size(est_map,2)), ...
    ones(size( est_map,2),1) * 50,"+r")

% Plot traj
draw_trajectory_pose(pos(:,end), quat(end,:), pos, quat, 2, 2, 'k', true)


xlabel("X (m)");
ylabel("Y (m)");
grid on
set(gca, 'Zdir', 'reverse')
set(gca, 'Ydir', 'reverse')
axis("equal")
xlim([min(truth_map{end,1}(1,:) - 5), max(truth_map{end,1}(1,:) + 5)])
ylim([min(truth_map{end,1}(2,:) - 5), max(truth_map{end,1}(2,:) + 5)])
view(0,90)