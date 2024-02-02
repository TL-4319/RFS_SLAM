function measurements = gen_pred_meas (pos, quat, landmark)
    pos_diff = landmark - pos;
    
    pos_diff_body_frame = rotateframe(quat, pos_diff');
    pos_diff_body_frame = pos_diff_body_frame';

    measurements = pos_diff_body_frame;
end