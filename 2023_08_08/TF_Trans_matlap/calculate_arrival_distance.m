function calculate_arrival_distance(base_link_pose, waypoint_id, waypoint_map_relative, waypoint_angle_map)
    dx = waypoint_map_relative(waypoint_id, 1) - base_link_pose.x;
    dy = waypoint_map_relative(waypoint_id, 2) - base_link_pose.y;
    delta_angle = rad2deg(atan2(dy, dx)) - waypoint_angle_map(waypoint_id);

    fprintf("dx = %6.3f\n", dx);
    fprintf("dy = %6.3f\n", dy);
    fprintf("waypoint_map_relative = %6.3f\n", waypoint_map_relative(waypoint_id, 1));
    fprintf("waypoint_map_relative = %6.3f\n", waypoint_map_relative(waypoint_id, 2));

    fprintf('delta angle = %6.3f\n', delta_angle);
    distance = sqrt(dx^2 + dy^2) * cosd(delta_angle);
    fprintf('cos distance = %6.3f\n', distance);
    % distance = sqrt(dx^2 + dy^2) * sind(delta_angle);
    % fprintf('sin distance = %6.3f\n', distance);
end
