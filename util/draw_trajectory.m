function draw_trajectory (pos, quat, traj_hist, axis_lenght, linewidth,traj_color,bool_hold)
    %% Function to draw a trajectory
    % pos - current position
    % quat - current quaternion
    % traj_hist - position vector from intial to previous timestep
    % axis_lenght - length of the drawn coordinate axis
    % linewidth - width of the coordinate frame axises
    % traj_color - color of trajectory. Set 'none' to disable
    % bool_hold - false if current draw is the first at current timestep.
    %           Will clear previous figure
    %           - true if draw_trajectory already been call at current
    %           timestep. Will no clear current figure
    % Tuan Luong - tdluong@crimson.ua.edu


    % Rotate of principle axis
    rot_x = rotatepoint(quat, [1,0,0]) * axis_lenght;
    rot_y = rotatepoint(quat, [0,1,0]) * axis_lenght;
    rot_z = rotatepoint(quat, [0,0,1]) * axis_lenght;

    unit_x = horzcat (pos, pos + rot_x');
    unit_y = horzcat (pos, pos + rot_y');
    unit_z = horzcat (pos, pos + rot_z');

    %figure (fig_num)
    if bool_hold
        hold on
    else
        hold off
    end
    plot3 (unit_x(1,:), unit_x(2,:),unit_x(3,:), 'r','LineWidth',linewidth, 'HandleVisibility','off')
    hold on
    plot3 (unit_y(1,:), unit_y(2,:),unit_y(3,:), 'g','LineWidth',linewidth,'HandleVisibility','off')
    plot3 (unit_z(1,:), unit_z(2,:),unit_z(3,:), 'b', 'LineWidth',linewidth,'HandleVisibility','off')
    if ~strcmp(traj_color,'none')
        plot3 (traj_hist(1,:), traj_hist (2,:), traj_hist(3,:), traj_color,'HandleVisibility','off')
    end
    hold off
end