% Simulation. The math is general for 3D but the simulation is limit to 2D
close all;
clear;
clc;

%% 

num_run = 100;
draw = false;
run_counter = 0;

monte_result = cell(num_run,1);
%% Load truth and measurement data
addpath ('../util/')

load('../dataset/truth_3D_15hz_dense.mat');

parfor monte_ind = 1:num_run
    disp(monte_ind)
    monte_result{monte_ind} = run_sim_instance(truth);
    
end
