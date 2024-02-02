function measurements = gen_meas (pos, quat, landmark, sensor)
    % Range check
    pos_diff = landmark - pos;
    dist = vecnorm(pos_diff,2,1);
    inrange_ind = find (dist < sensor.Range);
    landmark_inrange = landmark(:, inrange_ind);
    pos_diff_inrange = pos_diff(:,inrange_ind);
    
    % FOV check
    pos_diff_inrange_body_frame = rotateframe(quat, pos_diff_inrange');
    pos_diff_inrange_body_frame = pos_diff_inrange_body_frame';

    % Horizontal FOV
    landmark_bearing_from_sensor = atan2(pos_diff_inrange_body_frame(2,:),pos_diff_inrange_body_frame(1,:));
    in_FOV_ind = find(abs(landmark_bearing_from_sensor) < sensor.HFOV/2);
    body_meas_in_FOV = pos_diff_inrange_body_frame (:,in_FOV_ind);

    % Detection probability
    detected_prob = rand(1,size(body_meas_in_FOV,2));
    detected_ind = find (detected_prob < sensor.P_d);
    detected_meas_in_FOV = body_meas_in_FOV(:, detected_ind);

    % Sensor noise
    pos_error = normrnd (0, sensor.sigma, 3, size(detected_meas_in_FOV,2));
    pos_error(3,:) = zeros(1,size(pos_error,2));
    detected_meas_with_noise = detected_meas_in_FOV + pos_error;

    % Add clutter
    num_clutter = poissrnd (sensor.clutter_rate);
    clutter_range = rand(1,num_clutter) * sensor.Range;
    clutter_bearing = (rand(1,num_clutter) - 0.5) * sensor.HFOV/2;
    clutter_meas = zeros(3,num_clutter);
    clutter_meas(1,:) = cos(clutter_bearing) .* clutter_range;
    clutter_meas(2,:) = sin(clutter_bearing) .* clutter_range;
    cluttered_meas = horzcat (detected_meas_with_noise, clutter_meas);
    

    measurements = cluttered_meas;
end