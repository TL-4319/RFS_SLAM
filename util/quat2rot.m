function R = quat2rot(quat,rotation_frame)
    arguments
        quat (:,4) {mustBeNumeric}
        rotation_frame {mustBeMember(rotation_frame,["frame", "point"])}
    end

    % "frame" corresponds with inertial to body rotation
    % "point" corresponds with body to inertial rotation
    
    % orientation
    qw = quat(1);
    qx = quat(2);
    qy = quat(3);
    qz = quat(4);

    if rotation_frame == "point"
        qx = -qx;
        qy = -qy;
        qz = -qz;
    end

    R = zeros(3,3);

    R(1,1) = 1 - 2 * qy^2 - 2 * qz^2;
    R(1,2) = 2 * qx * qy + 2 * qw * qz;
    R(1,3) = 2 * qx * qz - 2 * qw * qy;

    R(2,1) = 2 * qx * qy - 2 * qw * qz;
    R(2,2) = 1 - 2 * qx^2 - 2 * qz^2;
    R(2,3) = 2 * qy * qz + 2 * qw * qx;

    R(3,1) = 2 * qx * qz + 2 * qw * qy;
    R(3,2) = 2 * qy * qz - 2 * qw * qx;
    R(3,3) = 1 - 2 * qx^2 - 2 * qy^2;

    
    
end
