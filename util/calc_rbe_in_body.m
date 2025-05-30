function [range, bearing, elevation] = calc_rbe_in_body (landmark,...
    sensor_pos, sensor_quat)
    
    pos_diff_in_body = calc_local_pos(sensor_pos, sensor_quat, landmark);

    % Range calc 
    range = vecnorm(pos_diff_in_body,2,1);
    
    % Calc bearing from sensor
    bearing = atan2(pos_diff_in_body(2,:),pos_diff_in_body(1,:));
   
    
    % Calc elevation from sensor
    r = (pos_diff_in_body(2,:).^2 + pos_diff_in_body(1,:).^2).^0.5;
    elevation = atan2(pos_diff_in_body(3,:),r);

end