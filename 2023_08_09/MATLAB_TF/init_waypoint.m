function [waypoint_utm_relative, utm_link_origin] = init_waypoint()
    waypoint_utm = [
        403165.834539,  4129142.66722;
        403163.074411,  4129144.66310;
        403168.006126,  4129155.70202;
        403171.552590,  4129159.62227;
        403175.499272,  4129163.65995;
        403188.494255,  4129175.23815;
        403192.579036,  4129177.42123;
        403195.456870,  4129178.70838;
        403196.654117,  4129182.60031;
        403188.253848,  4129179.74587;
        403183.344647,  4129175.25328;
        403178.839622,  4129171.98767;
        403174.786097,  4129167.14121;
        403172.102604,  4129165.03072;
        403167.683418,  4129166.12481
    ];

    waypoint_utm_datum = waypoint_utm(1, :);

    waypoint_utm_relative = zeros(size(waypoint_utm));
    for i = 1:size(waypoint_utm, 1)
        waypoint_utm_relative(i, :) = waypoint_utm(i, :) - waypoint_utm_datum;
        fprintf('utm : %2d  X : %6.3f  Y : %6.3f \n', i, waypoint_utm(i, 1), waypoint_utm(i, 2));
    end
    fprintf('\n\n');

    for i = 1:size(waypoint_utm_relative, 1)
        fprintf('utm(r) : %2d  X : %6.3f  Y : %6.3f \n', i, waypoint_utm_relative(i, 1), waypoint_utm_relative(i, 2));
    end

    utm_link_origin.x = 0.0;
    utm_link_origin.y = 0.0;
    utm_link_origin.yaw = -90.0;

end