function meas_in_world = reproject_meas (sensor_pos, sensor_quat, meas, sensor_params)
    if size(meas,2) == 0
        meas_in_world = [];
        return
    end

    if strcmp(sensor_params.meas_model,'cartesian')
        % Reproject measurements into world frame
        meas_rot_to_world = rotatepoint (quat, meas');
        meas_in_world = pos + meas_rot_to_world';
    elseif strcmp(sensor_params.meas_model,'range-bearing-elevation')
        % RBE to XYZ conversion
        XYZ_meas = zeros(size(meas));
        XYZ_meas(1,:) = meas(1,:) .* cos(meas(3,:)) .* cos(meas(2,:));
        XYZ_meas(2,:) = meas(1,:) .* cos(meas(3,:)) .* sin(meas(2,:));
        XYZ_meas(3,:) = meas(1,:) .* sin(meas(3,:));

        % Now reproject meas to world
        meas_rot_to_world = rotatepoint(sensor_quat, XYZ_meas');
        meas_in_world = sensor_pos + meas_rot_to_world';
    else
        error ("Invalid measurement model")
    end
end