function distance = calculate_distance_point_line_equation_map(in_point2d, waypoint_id, waypoint_line_equation_map)
    a = waypoint_line_equation_map(waypoint_id).a;
    b = waypoint_line_equation_map(waypoint_id).b;

    distance = abs(a * in_point2d.x + b - in_point2d.y) / sqrt(a^2 + 1);
end
