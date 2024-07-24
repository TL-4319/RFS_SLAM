function results = phd_slam1_2d_instance(dataset, sensor_params, odom_params, filter_params)
    addpath '../../util/'
    draw = false;

    time_vec = dataset.time_vec;
    dt = time_vec(2) - time_vec(1);

    %% Prepare truth data struct
    truth.pos = dataset.pos;
    truth.quat = dataset.quat;
    truth.cummulative_landmark_in_FOV = cell(size(time_vec,2),1);
    
    %% Pre allocate datas for estimation and odom est
    odom.pos = truth.pos;
    odom.quat = truth.quat;
    est.pos = truth.pos;
    est.quat = truth.quat;
    est.map_est = cell(size(time_vec,2),1);
    est.compute_time = zeros(size(time_vec,2),1);
    
    %% Initialize filter
    if strcmp(sensor_params.meas_model,'cartesian')
        [meas, ~,landmark_in_FOV] = gen_meas_cartesian(truth.pos(:,1),...
            truth.quat(1,:),dataset.landmark_locations, sensor_params);
        truth.cummulative_landmark_in_FOV{1,1} = landmark_in_FOV; 
    elseif strcmp(sensor_params.meas_model,'range-bearing')
    else
        error_msg = strcat(sensor_params.meas_model, " measurement model is not supported");
        error(error_msg);
    end
end