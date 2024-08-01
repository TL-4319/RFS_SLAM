function meas_in_world = reproject_meas (pos, quat, meas, sensor_params)
    % Reproject measurements into world frame
    if strcmp(sensor_params.meas_model,'cartesian')
        meas_rot_to_world = rotatepoint (quat, meas');
        meas_in_world = pos + meas_rot_to_world';
    elseif strcmp(sensor_params.meas_model,'range-bearing')
    else
    end
end