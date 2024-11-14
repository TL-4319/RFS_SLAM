function [meas, perfect_meas,landmark_in_FOV, PD_vec_multi] = ...
    gen_meas_cartesian_2D(pos, quat, landmark, sensor_params)
    % Created by Tuan Luong - 2024/08/01
    % Generate measurements (noisy and perfect) given current pose and map 
    
    %% FOV check
    % Return true landmark position within FOV in world frame
    % Return true landmark position within FOV in sensor frame as perfect
    % meas
    [perfect_meas, is_in_FOV, PD_vec_multi] = ...
        check_in_FOV_2D(landmark, pos, quat, sensor_params);
    landmark_in_FOV = landmark(:,is_in_FOV);

    %% Add noise, detection prob and clutter to measurements
    % Meas detect
    detected_prob = rand(1,size(perfect_meas,2));
    detected_ind = find(detected_prob < sensor_params.detect_prob);
    detected_meas = perfect_meas(:,detected_ind);

    % Meas noise
    meas_error = zeros(3,size(detected_meas,2));
    for i = 1:size(sensor_params.measurement_std,2)
        meas_error(i,:) = normrnd(0,sensor_params.measurement_std(i),1,size(detected_meas,2));
    end
    detected_noisy_meas = detected_meas + meas_error;

    % Add clutter that follow Poisson
    num_clutter = poissrnd (sensor_params.avg_num_clutter);
    clutter_range = rand(1,num_clutter) .* ...
        (sensor_params.max_range - sensor_params.min_range) + sensor_params.min_range;
    clutter_bearing = (rand(1,num_clutter) - 0.5) * sensor_params.HFOV/2;
    clutter_meas = zeros(3,num_clutter);
    clutter_meas(1,:) = cos(clutter_bearing) .* clutter_range;
    clutter_meas(2,:) = sin(clutter_bearing) .* clutter_range;

    meas = horzcat (detected_noisy_meas, clutter_meas);
end