function waypoint_angle_utm = calculate_waypoint_angle_utm(waypoint_utm_relative)
    waypoint_angle_utm = zeros(1, 15);
    
    for i = 2:15
        dx = waypoint_utm_relative(i, 1) - waypoint_utm_relative(i - 1, 1);
        dy = waypoint_utm_relative(i, 2) - waypoint_utm_relative(i - 1, 2);
        
        angle = atan2(dy, dx);
        waypoint_angle_utm(i) = rad2deg(angle);
        fprintf('wp[%2d] line angle(utm) %6.3f\n', i-1, rad2deg(angle));
    end
    fprintf('\n');
end
