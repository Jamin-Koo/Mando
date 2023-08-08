function [distance] = calculate_waypoint_distance_utm(waypoint_utm_relative)
    
    for i = 2:length(waypoint_utm_relative)
        dx = waypoint_utm_relative(i, 1) - waypoint_utm_relative(i - 1, 1);
        dy = waypoint_utm_relative(i, 2) - waypoint_utm_relative(i - 1, 2);
        
        distance(i) = sqrt(dx^2 + dy^2);
    end
    

end
