function draw_trajectory_hist (pos, quat, axis_lenght, linewidth,traj_color,bool_hold, step)
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
    rot_x = rotatepoint(quat(end,:), [1,0,0]) * axis_lenght;
    rot_y = rotatepoint(quat(end,:), [0,1,0]) * axis_lenght;
    rot_z = rotatepoint(quat(end,:), [0,0,1]) * axis_lenght;

    unit_x = horzcat (pos(:,end), pos(:,end) + rot_x');
    unit_y = horzcat (pos(:,end), pos(:,end) + rot_y');
    unit_z = horzcat (pos(:,end), pos(:,end) + rot_z');

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
        plot3 (pos(1,:), pos (2,:), pos(3,:), traj_color,'HandleVisibility','off',LineWidth=2)
    end
    
    for ii = 1:step:size(pos,2)-step
        % Rotate of principle axis
        rot_x = rotatepoint(quat(ii,:), [1,0,0]) * axis_lenght/1.5;
        rot_y = rotatepoint(quat(ii,:), [0,1,0]) * axis_lenght/1.5;
        rot_z = rotatepoint(quat(ii,:), [0,0,1]) * axis_lenght/1.5;
    
        unit_x = horzcat (pos(:,ii), pos(:,ii) + rot_x');
        unit_y = horzcat (pos(:,ii), pos(:,ii) + rot_y');
        unit_z = horzcat (pos(:,ii), pos(:,ii) + rot_z');
        plot3 (unit_x(1,:), unit_x(2,:),unit_x(3,:), 'r','LineWidth',linewidth, 'HandleVisibility','off')
        plot3 (unit_y(1,:), unit_y(2,:),unit_y(3,:), 'g','LineWidth',linewidth,'HandleVisibility','off')
        plot3 (unit_z(1,:), unit_z(2,:),unit_z(3,:), 'b', 'LineWidth',linewidth,'HandleVisibility','off')
    end

    hold off
end