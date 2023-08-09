function distance = calculate_waypoint_arrival_distance(base_link_pose, waypoint_id, waypoint_map_relative, waypoint_angle_map)
    dx = waypoint_map_relative(waypoint_id, 1) - base_link_pose.x;   % 목표점 - 시작점  X 성분
    dy = waypoint_map_relative(waypoint_id, 2) - base_link_pose.y;   % 목표점 - 시작점  Y 성분

    delta_angle = rad2deg(atan2(dx, dy)) - waypoint_angle_map(waypoint_id);

    fprintf("delta_angle = %6.3f\n", delta_angle);

    distance = sqrt(dx^2 + dy^2) * cos(rad2deg(delta_angle));

    fprintf("cos distance = %6.3f\n", distance);
    %distance = sqrt(dx^2 + dy^2) * sind(delta_angle);
    %fprintf("sin distance = %6.3f\n", distance);
end
