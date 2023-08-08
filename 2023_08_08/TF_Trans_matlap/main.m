clc
clear

% Define a struct named 'Point2D' with 'x' and 'y' fields
base_link_origin = struct('x', 1.0, 'y', 1.0, 'yaw', 45);

% Call the Initialization Function
[waypoint_utm_relative, utm_link_origin] = initialize_waypoint();

% Display the results of initialization
disp('Waypoint UTM Relative:');
disp(waypoint_utm_relative);


disp('UTM Link Origin:');
disp(utm_link_origin);


% Calculate and store the rotation matrix
Rotation_matrix_axis = set_rotation_matrix_axis(-utm_link_origin.yaw);

disp('Rotation_matrix_axis:');
disp(Rotation_matrix_axis);

waypoint_utm_relative_x = waypoint_utm_relative(1,:);
waypoint_utm_relative_y = waypoint_utm_relative(2,:);

disp('Waypoint UTM Relative XX:');
disp(waypoint_utm_relative_x);
disp('Waypoint UTM Relative YY:');
disp(waypoint_utm_relative_y);

% Transform UTM coordinates to Map coordinates for each waypoint

%for i = 1:size(waypoint_utm_relative, 1)
%    [waypoint_map_relative_x(i)] = TF_utm_map_X(waypoint_utm_relative_x(i), waypoint_utm_relative_y(i), Rotation_matrix_axis);
%end

[waypoint_map_relative] = TF_utm_map_X(waypoint_utm_relative, Rotation_matrix_axis);

% Display the computed map coordinates
disp('Waypoint Map Relative:');
disp(waypoint_map_relative);


% Calculate and display waypoint angles in UTM
[utm_line_angle] = calculate_waypoint_angle_utm(waypoint_utm_relative);
disp("utm_line_angle : ");
disp(utm_line_angle);


% Calculate and display waypoint angles in MAP
[map_line_angle] = calculate_waypoint_angle_utm(waypoint_map_relative);
disp("map_line_angle : ");
disp(map_line_angle);

% Calculate and display waypoint distance in UTM
[utm_line_distance] = calculate_waypoint_distance_utm(waypoint_utm_relative);
disp("utm_line_distance : ");
disp(utm_line_distance);

% Calculate and display waypoint equation in UTM
calculate_waypoint_equation_utm(waypoint_utm_relative);

% Calculate and display waypoint equation in MAP
calculate_waypoint_equation_map(waypoint_map_relative);

waypoint_id = 2;
calculate_arrival_distance(base_link_origin, waypoint_id, waypoint_map_relative, map_line_angle);


