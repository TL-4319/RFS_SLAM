function Sq = quat2Sq (quat)
    % Calculate the jacobian of quaternion dot wrt to rotational velocity 
    
    % orientation
    qw = quat(1);
    qx = quat(2);
    qy = quat(3);
    qz = quat(4);

    Sq = zeros(4,3);
    Sq(1,1) = -qx;
    Sq(1,2) = -qy;
    Sq(1,3) = -qz;

    Sq(2,1) = qw;
    Sq(2,2) = qz;
    Sq(2,3) = -qy;

    Sq(3,1) = -qz;
    Sq(3,2) = qw;
    Sq(3,3) = qx;

    Sq(4,1) = qy;
    Sq(4,2) = -qx;
    Sq(4,3) = qw;
end