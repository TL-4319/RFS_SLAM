function [measurements, landmark_in_fov] = gen_noisy_3D_meas (pos, quat, landmark, sensor)
    [body_meas_in_FOV, is_in_FOV] = check_in_FOV_3D(landmark, pos, quat, sensor);

    landmark_in_fov = landmark(:,is_in_FOV);
    % Detection probability
    detected_prob = rand(1,size(body_meas_in_FOV,2));
    detected_ind = find (detected_prob < sensor.P_d);
    detected_meas_in_FOV = body_meas_in_FOV(:, detected_ind);

    % Sensor noise
    pos_error = normrnd (0, sensor.sigma, 3, size(detected_meas_in_FOV,2));
    detected_meas_with_noise = detected_meas_in_FOV + pos_error;

    % Add clutter
    num_clutter = poissrnd (sensor.clutter_rate);
    clutter_range = rand(1,num_clutter) .* (sensor.max_range - sensor.min_range) + sensor.min_range;
    clutter_bearing = (rand(1,num_clutter) - 0.5) * sensor.HFOV/2;
    clutter_elevation = (rand(1,num_clutter) - 0.5) * sensor.VFOV/2;
    clutter_meas(1,:) = cos(clutter_bearing) .* clutter_range;
    clutter_meas(2,:) = sin(clutter_bearing) .* clutter_range;
    clutter_meas(3,:) = sin(clutter_elevation) .* clutter_range;
    cluttered_meas = horzcat (detected_meas_with_noise, clutter_meas);
    

    measurements = cluttered_meas;
end