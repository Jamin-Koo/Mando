function [angle] = calculate_waypoint_angle_utm(waypoint_utm_relative)

    for i = 2:size(waypoint_utm_relative, 1)
        dx = waypoint_utm_relative(i, 1) - waypoint_utm_relative(i - 1, 1);
        dy = waypoint_utm_relative(i, 2) - waypoint_utm_relative(i - 1, 2);

        angle(i) = atan2(dy, dx) * (180 / pi);
    end
    %disp(angle);
end
