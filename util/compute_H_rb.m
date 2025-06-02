function H = compute_H_rb(sensor_pos, sensor_quat, temp_mu)
    H = zeros(2,2,size(temp_mu,2));
    
    mu_in_sensor = calc_local_pos(sensor_pos, sensor_quat, temp_mu);
    R = quat2rot(compact(sensor_quat),"frame");

    mu_in_sensor_squared = mu_in_sensor.^2;
    r2_squared = sum(mu_in_sensor_squared(1:2,:),1); r2 = r2_squared.^0.5;
    
    for ii = 1:size(temp_mu,2)
        % H(1,1,ii) = (R(1,1) * mu_in_sensor(1,ii) + ...
        %     R(2,1) * mu_in_sensor(2,ii)) / r2(ii);
        % H(1,2,ii) = (R(1,2) * mu_in_sensor(1,ii) + ...
        %     R(2,2) * mu_in_sensor(2,ii) ) / r2(ii);
        % 
        % H(2,1,ii) = (R(2,1) * mu_in_sensor(1,ii) - ...
        %     R(1,1) * mu_in_sensor(2,ii)) / r2_squared(ii);
        % H(2,2,ii) = (R(2,2) * mu_in_sensor(1,ii) - ...
        %     R(1,2) * mu_in_sensor(2,ii)) / r2_squared(ii);

        H(1,1,ii) = (temp_mu(1,ii) - sensor_pos(1)) / r2(ii);
        H(1,2,ii) = (temp_mu(2,ii) - sensor_pos(2)) / r2(ii);

        H(2,1,ii) = - (temp_mu(2,ii) - sensor_pos(2)) / r2_squared(ii);
        H(2,2,ii) = (temp_mu(1,ii) - sensor_pos(1)) / r2_squared(ii);
    end
end