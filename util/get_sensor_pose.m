function [sensor_pos, sensor_quat] = get_sensor_pose(base_pos, base_quat, sensor_params)
    sensor_pos = base_pos + transpose(rotatepoint(base_quat,sensor_params.pos_body_sensor'));
    sensor_quat = quatmultiply(base_quat, sensor_params.quat_body_sensor);
end