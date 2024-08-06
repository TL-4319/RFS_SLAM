function [pos_diff_in_body, is_in_FOV, PD_vec_multiplier] = check_in_FOV_2D(landmark, pos, quat, sensor)
    % Range calc
    pos_diff = landmark - pos;
    range = vecnorm(pos_diff,2,1);
    
    % Calc bearing from sensor
    pos_diff_in_body = rotateframe(quat, pos_diff'); % Rotate pos vector from global to sensor frame
    pos_diff_in_body = pos_diff_in_body';
    landmark_bearing_from_sensor = atan2(pos_diff_in_body(2,:),pos_diff_in_body(1,:));

    % Find landmark within the FOV
    is_FOV = ~any([abs(landmark_bearing_from_sensor) > sensor.HFOV/2 ;...
            range > sensor.max_range ; range < sensor.min_range],1);
    
    % Return only landmark within FOV
    pos_diff_in_body = pos_diff_in_body(:,is_FOV);
    is_in_FOV = is_FOV;
    range_in_FOV = range(:,is_FOV);
    landmark_bearing_from_sensor_in_FOV = landmark_bearing_from_sensor(:,is_FOV);
    
    PD_vec_multiplier = ones(1,size(pos_diff_in_body,2));
    if sensor.near_edge_PD_mult ~= 0.1 
    % Calculate P_D mult based on pos in FOV
    is_near_edge = any([abs(landmark_bearing_from_sensor_in_FOV) > (sensor.HFOV/2 - deg2rad(5)) ;...
            range_in_FOV > (sensor.max_range - 1) ; range_in_FOV < (sensor.min_range + 1)],1);
    PD_vec_multiplier(:, is_near_edge) = 0.2;
    end
end