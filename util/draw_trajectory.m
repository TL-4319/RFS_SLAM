function draw_trajectory (pos, quat, traj_hist,fig_num, scale, linewidth,traj_color,bool_hold)
    % Rotate of principle axis
    rot_x = rotatepoint(quat, [1,0,0]) * scale;
    rot_y = rotatepoint(quat, [0,1,0]) * scale;
    rot_z = rotatepoint(quat, [0,0,1]) * scale;

    unit_x = horzcat (pos, pos + rot_x');
    unit_y = horzcat (pos, pos + rot_y');
    unit_z = horzcat (pos, pos + rot_z');

    %figure (fig_num)
    if bool_hold
        hold on
    else
        hold off
    end
    plot3 (unit_x(1,:), unit_x(2,:),unit_x(3,:), 'r','LineWidth',linewidth)
    hold on
    plot3 (unit_y(1,:), unit_y(2,:),unit_y(3,:), 'g','LineWidth',linewidth)
    plot3 (unit_z(1,:), unit_z(2,:),unit_z(3,:), 'b', 'LineWidth',linewidth)
    plot3 (traj_hist(1,:), traj_hist (2,:), traj_hist(3,:), traj_color)
    hold off
end