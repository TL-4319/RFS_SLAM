function pos_in_local = calc_local_pos(local_pos, local_quat, pos_in_global)
    pos_diff_global = pos_in_global - local_pos;
    pos_in_local = rotateframe(local_quat, pos_diff_global');
    pos_in_local = pos_in_local';
end