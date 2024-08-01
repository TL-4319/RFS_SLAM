% Run monte carlo 2D simulations. Fundamental data and formulation is 3D 
% dynamics but constraints are added to restrict motions to 2D
close all;
clear;
clc;

% Define number of MC runs
num_run = 1;

% Visualization
draw = true;

% add path to util functions. 
addpath('../../util/')

% Select dataset
load ('../generated_datasets/dataset_2d_30Hz.mat');

%% Define sensor parameters to be used to generate measurements
% For cartesian model, meas_vector = [x, y]'. 
% For range_bearing, meas_vector = [az, range]', 
sensor_params.meas_model = 'cartesian'; %[cartesian, range-bearing]
sensor_params.HFOV = deg2rad(100);
sensor_params.max_range = 15;
sensor_params.min_range = 0.4;
sensor_params.detect_prob = 0.8;
sensor_params.measurement_std = [0.1, 0.1]; 
sensor_params.avg_num_clutter = 3;

sensor_params.meas_area = sensor_params.HFOV * 0.5 * ...
    (sensor_params.max_range - sensor_params.min_range)^2;
sensor_params.clutter_density = sensor_params.avg_num_clutter / ...
    sensor_params.meas_area;

%% Define odometry configurations
odom_params.motion_sigma = [0.5; 0.5; 0.2]; 

%% Defind filter parameters
% Sensor params exposed to filter
filter_params.sensor = sensor_params; % Copy sensor parameter set so filter has different parameters for robust analysis
filter_params.sensor.detect_prob = 0.8;
filter_params.sensor.measurement_std = [0.1, 0.1];
filter_params.sensor.avg_num_clutter = 3;

% Particle filter params
filter_params.num_particle = 1;
filter_params.resample_threshold = 0.4; % Percentage of num_particle for resample to trigger
filter_params.likelihood_method = 'single-cluster'; %['empty', 'single-feature, 'single-cluster']

% Motion covariance = [cov_x, cov_y, cov_z, cov_phi, cov_theta, cov_psi]
% For 2D, cov_z, cov_phi and cov_theta = 0
filter_params.motion_model = 'truth'; % [odometry, random-walk, truth]
filter_params.motion_sigma = [0.5; 0.5; 0.2];

% Map PHD config
filter_params.birthGM_intensity = 0.1;             % Default intensity of GM component when birth
filter_params.birthGM_std = 0.05;                  % Default standard deviation in position of GM component when birth
filter_params.map_std = 0.0;
filter_params.adaptive_birth_dist_thres = 1;
filter_params.GM_inten_thres = 0.0;                % Threshold to use a component for importance weight calc and plotting
filter_params.pruning_thres = 10^-3;
filter_params.merge_dist = 5;
filter_params.num_GM_cap = 5000;
filter_params.inner_filter = 'ekf';

% NO INPUT REQUIRED for the rest of the section
% Calculate corresponding matrices
filter_params.sensor.clutter_density = filter_params.sensor.avg_num_clutter / ...
    filter_params.sensor.meas_area;
filter_params.sensor.R = diag(filter_params.sensor.measurement_std.^2);
filter_params.birthGM_cov = diag([filter_params.birthGM_std, filter_params.birthGM_std].^2);
filter_params.map_Q = diag([filter_params.map_std, filter_params.map_std].^2);

%% Setup struct to save datas
simulation.sensor_params = sensor_params;
simulation.filter_params = filter_params;
simulation.odom_params = odom_params;

results = cell(num_run,1);

for ii = 1:num_run
    results{ii,1} = phd_slam1_2d_instance(dataset,sensor_params, odom_params, filter_params, draw);
end