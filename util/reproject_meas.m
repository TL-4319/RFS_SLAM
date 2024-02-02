function meas_world = reproject_meas (pos, quat, meas)
    % Reproject measurements into world frame for verification
    meas_rot_to_world = rotatepoint (quat, meas');

    meas_world = pos + meas_rot_to_world';
end