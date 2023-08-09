function [waypoint_line_equation_utm] = calculate_waypoint_equation_utm(waypoint_utm_relative)
    waypoint_line_equation_utm = struct('a', zeros(1, 15), 'b', zeros(1, 15));

    for i = 2:15
        dx = waypoint_utm_relative(i, 1) - waypoint_utm_relative(i - 1, 1);
        dy = waypoint_utm_relative(i, 2) - waypoint_utm_relative(i - 1, 2);

        a = dy / dx;
        b = waypoint_utm_relative(i, 2) - a * waypoint_utm_relative(i, 1);

        waypoint_line_equation_utm(i).a = a;
        waypoint_line_equation_utm(i).b = b;

        fprintf('wp[%2d] waypoint equation(utm) y = %.3f * x + %.3f [m]\n', i-1, waypoint_line_equation_utm(i).a, waypoint_line_equation_utm(i).b);
    end
    fprintf('\n');
end
