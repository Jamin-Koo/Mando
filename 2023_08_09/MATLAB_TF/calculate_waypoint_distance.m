function waypoint_distance = calculate_waypoint_distance(waypoint)
    waypoint_distance = zeros(1, size(waypoint, 1) - 1);

    for i = 2:size(waypoint, 1)
        dx = waypoint(i, 1) - waypoint(i - 1, 1);
        dy = waypoint(i, 2) - waypoint(i - 1, 2);

        distance = sqrt(dx^2 + dy^2);
        waypoint_distance(i) = distance;

        fprintf('wp[%2d] waypoint distance: %6.3f [m]\n', i-1, distance);
    end

    fprintf('\n');
end
