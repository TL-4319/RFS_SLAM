function [is_in_FOV, PD_vec_multiplier] = ...
    check_in_FOV_3D(landmark, sensor_pos, sensor_quat, sensor)
    
    [range, landmark_bearing_from_sensor, landmark_elevation_from_sensor] = ...
        calc_rbe_in_body (landmark, sensor_pos, sensor_quat);
    
    % If VFOV just has 1 element, use constant VFOV. 
    % Multiple VFOV can be used to simulate Lissajous pattern
    if size(sensor.VFOV,1) == 1 
        temp = [abs(landmark_bearing_from_sensor) > sensor.HFOV/2 ;...
                range > sensor.max_range ; range < sensor.min_range; ...
                abs(landmark_elevation_from_sensor) > sensor.VFOV/2];
        is_in_FOV = ~any(temp,1);
    end


    PD_vec_multiplier = ones(1,size(range,2));
end