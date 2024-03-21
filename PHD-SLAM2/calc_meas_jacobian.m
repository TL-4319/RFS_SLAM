function H = calc_meas_jacobian(pos, quat, landmark_pos)
    % Augmented state vector of [landmark, pos, quat]
    % Innovation vector
    pos = reshape(pos,[1,3]);
    landmark_pos = reshape(landmark_pos,[1,3]);
    innov_vec = landmark_pos - pos;
    innovx = innov_vec(1);
    innovy = innov_vec(2);
    innovz = innov_vec(3);
    % ego orientation
    qw = quat(1);
    qx = quat(2);
    qy = quat(3);
    qz = quat(4);

    % Measurement model: h(l,p,quat) = quat2rot(quat) * (l-p)
    H = zeros(3,10);
    
    dhdl = quat2rot(quat,"frame");
    H(1:3,1:3) = dhdl;
    
    dhdp = -dhdl;
    H(1:3,4:6) = dhdp;
    
    H(1,7) = 2 * qz * innovy - 2 * qy * innovz;
    H(1,8) = 2 * qy * innovy + 2 * qz * innovz;
    H(1,9) = -4 * qy * innovx + 2 * qx * innovy - 2 * qw * innovz;
    H(1,10) = -4 * qz * innovx + 2 * qw * innovy + 2 * qx * innovz;

    H(2,7) = -2 * qz * innovx + 2 * qx * innovz;
    H(2,8) = 2 * qy * innovx - 4 * qx * innovy + 2 * qw * innovz;
    H(2,9) = 2 * qx * innovx + 2 * qz * innovz;
    H(2,10) = -2 * qw * innovx - 4 * qz * innovy + 2 * qy * innovz;
    
    H(3,7) = 2 * qy * innovx - 2 * qx * innovy;
    H(3,8) = 2 * qz * innovx - 2 * qw * innovy - 4 * qx * innovz;
    H(3,9) = 2 * qw * innovx + 2 * qz * innovy - 4 * qy * innovz;
    H(3,10) = 2 * qx * innovx + 2 * qy * innovy;
 end