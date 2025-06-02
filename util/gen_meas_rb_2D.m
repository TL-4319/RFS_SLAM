function [meas, perfect_meas,landmark_in_FOV, PD_vec_multi] = ...
    gen_meas_rb_2D(sensor_pos, sensor_quat, landmark, sensor_params)
    % Created by Tuan Luong - 2024/11/14
    % Generate range bearing elevation measurements (noisy and perfect) given current pose and map 

    %% FOV Check
    % Return true landmark position within FOV in world frame
    [is_in_FOV, PD_vec_multi] = ...
        check_in_FOV_2D(landmark, sensor_pos, sensor_quat, sensor_params);
    landmark_in_FOV = landmark(:,is_in_FOV);
    
    perfect_meas = zeros(2,size(landmark_in_FOV,2));

    [perfect_meas(1,:), perfect_meas(2,:), ~] = ...
        calc_rbe_in_body(landmark_in_FOV, sensor_pos, sensor_quat);

    %% Add noise, detection prob and clutter to measurement
    % Meas detect
    detected_prob = rand(1,size(perfect_meas,2));
    detected_ind = find(detected_prob < sensor_params.detect_prob);
    detected_meas = perfect_meas(:,detected_ind);
    
    % Meas noise assuming AWGN
    meas_error = zeros(2,size(detected_meas,2));
    for ii = 1:size(sensor_params.measurement_std,2)
        meas_error(ii,:) = normrnd(0, sensor_params.measurement_std(ii), 1, size(detected_meas,2));
    end
    detected_noisy_meas = detected_meas + meas_error;
    
    % Add clutter that follow Poisson distribution. Clutter also tend to
    % snap to ground to simulate false ground detections
    num_clutter = poissrnd (sensor_params.avg_num_clutter);
    
    clutter_range = rand(1,num_clutter) .* ...
        (sensor_params.max_range - sensor_params.min_range) + sensor_params.min_range;
    clutter_bearing = (rand(1,num_clutter) - 0.5) * sensor_params.HFOV/2;

    clutter_meas = vertcat(clutter_range, clutter_bearing);

    meas = horzcat(detected_noisy_meas, clutter_meas);

end