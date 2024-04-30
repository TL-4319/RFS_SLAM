function [pos_diff_in_body, is_FOV] = check_in_FOV_3D(landmark, pos, quat, sensor)
    % Range calc
    pos_diff = landmark - pos;
    range = vecnorm(pos_diff,2,1);
    
    % Calc bearing from sensor
    pos_diff_in_body = rotateframe(quat, pos_diff');
    pos_diff_in_body = pos_diff_in_body';
    landmark_bearing_from_sensor = atan2(pos_diff_in_body(2,:),pos_diff_in_body(1,:));

    % Calc elevation from sensor
    landmark_elevation_from_sensor = atan2(pos_diff_in_body(3,:),pos_diff_in_body(1,:));
    
    temp = [abs(landmark_bearing_from_sensor) > sensor.HFOV/2 ;...
            range > sensor.max_range ; range < sensor.min_range; ...
            abs(landmark_elevation_from_sensor) > sensor.VFOV/2];
    is_FOV = ~any(temp,1);
    % is_FOV = ~any([abs(landmark_bearing_from_sensor) > sensor.HFOV/2 ;...
    %         range > sensor.max_range ; range < sensor.min_range; ...
    %         abs(landmark_elevation_from_sensor) > sensor.VFOV/2],1);
    pos_diff_in_body = pos_diff_in_body(:,is_FOV);
end