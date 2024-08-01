function [pos_diff_in_body, is_in_FOV] = check_in_FOV_2D(landmark, pos, quat, sensor)
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
end