function [F, Q] = calc_pred_jacobian(pos, quat, v, omega, dt)
    % position
    x = pos(1);
    y = pos(2);
    z = pos(3);
    % orientation
    qw = quat(1);
    qx = quat(2);
    qy = quat(3);
    qz = quat(4);
    % translational vel
    vx = v(1);
    vy = v(2);
    vz = v(3);
    % rotational vel
    wx = omega(1);
    wy = omega(2);
    wz = omega(3);

    % Motion model
    % pos = prev_pos + quat2rot(quat) * v * dt
    % quat = (I + 0.5 * omega * dt) * prev_quat
    % Construct F matrix
    F = eye(7);

    F(1,4) = -2 * qz * dt * vy + 2 * qy * dt * vz;
    F(1,5) = 2 * qy * dt * vy + 2 * qz * dt * vz;
    F(1,6) = -4 * qy * dt * vx + 2 * qx * dt * vy + 2 * qw * dt * vz;
    F(1,7) = -4 * qz * dt * vx - 2 * qw * dt * vy + 2 * qx * dt * vz;

    F(2,4) = 2 * qz * dt * vx - 2 * qx * dt * vz;
    F(2,5) = 2 * qy * dt * vx - 4 * qx * dt * vy - 2 * qw * dt * vz;
    F(2,6) = 2 * qx * dt * vx + 2 * qz * dt * vz;
    F(2,7) = 2 * qw * dt * vx - 4 * qz * dt * vy + 2 * qy * dt * vz;

    F(3,4) = -2 * qy * dt * vx + 2 * qx * dt * vy;
    F(3,5) = 2 * qz * dt * vx + 2 * qw * dt * vy - 4 * qx * dt * vz;
    F(3,6) = -2 * qw * dt * vx + 2 * qz * dt * vy - 4 * qy * dt * vz;
    F(3,7) = 2 * qx * dt * vx + 2 * qy * dt * vy;

    F(4,5) = -0.5 * dt * wx;
    F(4,6) = -0.5 * dt * wy;
    F(4,7) = -0.5 * dt * wz;

    F(5,4) = 0.5 * dt * wx;
    F(5,6) = 0.5 * dt * wz;
    F(5,7) = -0.5 * dt * wy;

    F(6,4) = 0.5 * dt * wy;
    F(6,5) = -0.5 * dt * wz;
    F(6,7) = 0.5 * dt * wx;

    F(7,4) = 0.5 * dt * wz;
    F(7,5) = 0.5 * dt * wy;
    F(7,6) = -0.5 * dt * wx;

    % Construct Q matrix
    Q = zeros(7,6);

    Q(1:3,1:3) = quat2rot(quat, "point") * dt;
    
    Q(4,4) = 0.5 * dt * qx;
    Q(4,5) = -0.5 * dt * qy;
    Q(4,6) = -0.5 * dt * qz;

    Q(5,4) = 0.5 * dt * qw;
    Q(5,5) = -0.5 * dt * qz;
    Q(5,6) = 0.5 * dt * qy;
    
    Q(6,4) = 0.5 * dt * qz;
    Q(6,5) = 0.5 * dt * qw;
    Q(6,6) = -0.5 * dt * qx;
    
    Q(7,4) = -0.5 * dt * qy;
    Q(7,5) = 0.5 * dt * qx;
    Q(7,6) = 0.5 * dt * qw;

end