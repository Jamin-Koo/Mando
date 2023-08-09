function waypoint_line_equation_map = calculate_waypoint_equation_map(waypoint_map_relative)
    waypoint_line_equation_map = struct('a', [], 'b', []);

    for i = 2:size(waypoint_map_relative, 1)
        dx = waypoint_map_relative(i, 1) - waypoint_map_relative(i - 1, 1);
        dy = waypoint_map_relative(i, 2) - waypoint_map_relative(i - 1, 2);
        a = dy / dx;
        b = waypoint_map_relative(i, 2) - a * waypoint_map_relative(i, 1);
        waypoint_line_equation_map(i).a = a;
        waypoint_line_equation_map(i).b = b;

        fprintf('wp[%2d] waypoint equation(map) y = %.3f * x + %.3f [m]\n', i-1, waypoint_line_equation_map(i).a, waypoint_line_equation_map(i).b);
    end
    fprintf('\n');
end
