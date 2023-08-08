function calculate_waypoint_equation_utm(waypoint_utm_relative)
    waypoint_line_equation_utm = cell(1, length(waypoint_utm_relative) - 1);
    
    fprintf('\n');
    
    for i = 2:length(waypoint_utm_relative)
        dx = waypoint_utm_relative(i, 1) - waypoint_utm_relative(i - 1, 1);
        dy = waypoint_utm_relative(i, 2) - waypoint_utm_relative(i - 1, 2);
        
        a = dy / dx;
        b = waypoint_utm_relative(i, 2) - a * waypoint_utm_relative(i, 1);
        
        waypoint_line_equation_utm{i}.a = a;
        waypoint_line_equation_utm{i}.b = b;
        
        fprintf('wp[%2d] waypoint equation(utm) y = %6.3f * x + %6.3f   [m]\n', i, a, b);
    end
    
    fprintf('\n');
end
