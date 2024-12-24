function [meas, perfect_meas,landmark_in_FOV, PD_vec_multi] = ...
    gen_meas_rbe_3D(sensor_pos, sensor_quat, landmark, sensor_params)
    % Created by Tuan Luong - 2024/11/14
    % Generate range bearing elevation measurements (noisy and perfect) given current pose and map 

    %% FOV Check
    % Return true landmark position within FOV in world frame
    [is_in_FOV, PD_vec_multi] = ...
        check_in_FOV_3D(landmark, sensor_pos, sensor_quat, sensor_params);
    landmark_in_FOV = landmark(:,is_in_FOV);
    
    perfect_meas = zeros(3,size(landmark_in_FOV,2));

    [perfect_meas(1,:), perfect_meas(2,:), perfect_meas(3,:)] = ...
        calc_rbe_in_body(landmark_in_FOV, sensor_pos, sensor_quat);

    %% Add noise, detection prob and clutter to measurement
    % Meas detect
    detected_prob = rand(1,size(perfect_meas,2));
    detected_ind = find(detected_prob < sensor_params.detect_prob);
    detected_meas = perfect_meas(:,detected_ind);
    
    % Meas noise assuming AWGN
    meas_error = zeros(3,size(detected_meas,2));
    for ii = 1:size(sensor_params.measurement_std,2)
        meas_error(ii,:) = normrnd(0, sensor_params.measurement_std(ii), 1, size(detected_meas,2));
    end
    detected_noisy_meas = detected_meas + meas_error;
    
    % Add clutter that follow Poisson distribution. Clutter also tend to
    % snap to ground to simulate false ground detections
    num_clutter = poissrnd (sensor_params.avg_num_clutter);
    
    clutter_bearing = (rand(1,num_clutter) - 0.5) * sensor_params.HFOV/2;
    clutter_elevation = (rand(1,num_clutter) - 0.5) * sensor_params.VFOV/2;

    clutter_detection_z_world = (rand(1,num_clutter) - 0.5) * 0.1;
    clutter_diff_world = clutter_detection_z_world - sensor_pos(3);
    sensor_euler = quat2eul(sensor_quat, "ZYX");
    clutter_el_in_world = sensor_euler(2) - clutter_elevation;
    clutter_range = clutter_diff_world ./ abs(sin(clutter_el_in_world));

    clutter_meas = vertcat(clutter_range, clutter_bearing, clutter_elevation);

    meas = horzcat(detected_noisy_meas, clutter_meas);

end