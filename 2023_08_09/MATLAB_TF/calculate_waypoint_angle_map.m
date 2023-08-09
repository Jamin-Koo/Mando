function waypoint_angle_map = calculate_waypoint_angle_map(waypoint_map_relative)
    waypoint_angle_map = zeros(1, 15);  % Assuming there are 15 waypoints

    for i = 2:15
        dx = waypoint_map_relative(i, 1) - waypoint_map_relative(i - 1, 1);
        dy = waypoint_map_relative(i, 2) - waypoint_map_relative(i - 1, 2);

        angle = atan2(dy, dx);
        waypoint_angle_map(i) = rad2deg(angle);

        fprintf('wp[%2d] line angle(map) %6.3f\n', i-1, rad2deg(angle));
    end

    fprintf('\n');
end
