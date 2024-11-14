function [pos_diff_in_body, rbe_in_body, is_in_FOV, PD_vec_multiplier] = check_in_FOV_3D(landmark, pos, quat, sensor)
    % Range calc
    pos_diff = landmark - pos;
    range = vecnorm(pos_diff,2,1);
    
    % Calc bearing from sensor
    pos_diff_in_body = rotateframe(quat, pos_diff');
    pos_diff_in_body = pos_diff_in_body';
    landmark_bearing_from_sensor = atan2(pos_diff_in_body(2,:),pos_diff_in_body(1,:));

    % Calc elevation from sensor
    landmark_elevation_from_sensor = atan2(pos_diff_in_body(3,:),pos_diff_in_body(1,:));

    % If VFOV just has 1 element, use constant VFOV. 
    % Multiple VFOV can be used to simulate Lissajous pattern
    if size(sensor.VFOV,1) == 1 
        temp = [abs(landmark_bearing_from_sensor) > sensor.HFOV/2 ;...
                range > sensor.max_range ; range < sensor.min_range; ...
                abs(landmark_elevation_from_sensor) > sensor.VFOV/2];
        is_in_FOV = ~any(temp,1);
        % is_FOV = ~any([abs(landmark_bearing_from_sensor) > sensor.HFOV/2 ;...
        %         range > sensor.max_range ; range < sensor.min_range; ...
        %         abs(landmark_elevation_from_sensor) > sensor.VFOV/2],1);
    end
    pos_diff_in_body = pos_diff_in_body(:,is_in_FOV);

    % Make RBE meas vector as well
    rbe_in_body = vertcat(range(:,is_in_FOV), landmark_bearing_from_sensor(:,is_in_FOV),...
        landmark_elevation_from_sensor(:,is_in_FOV));

    PD_vec_multiplier = ones(1,size(rbe_in_body,2));
end