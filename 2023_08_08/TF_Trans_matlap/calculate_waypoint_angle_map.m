function [angle] = calculate_waypoint_angle_map(waypoint_map_relative)

    for i = 2:size(waypoint_map_relative, 1)
        dx = waypoint_map_relative(i, 1) - waypoint_map_relative(i - 1, 1);
        dy = waypoint_map_relative(i, 2) - waypoint_map_relative(i - 1, 2);

        angle(i) = atan2(dy, dx) * (180 / pi);
    end
    %disp(angle);
end
