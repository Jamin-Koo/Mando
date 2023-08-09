clc, clear;
%%%%%%%%%%%%%%%%%%%%%%%%% initialize variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%
Point2D = struct('x', 0.0, 'y', 0.0);
Pose2D = struct('x', 0.0, 'y', 0.0, 'yaw', 0.0);

cross_track_error_utm = 0.0;
cross_track_error_map = 0.0;
heading_angle_error = 0.0;    % heading angle 의 기존은 map의 north 가 0 이며 시계반대 방향(CCW)
cross_track_error_angle = 0.0;
ctr_factor = 1.0;
target_steering_angle = 0.0;
current_waypoint_id = 2;       % 현재 목표점 waypoint ID

%%%%%%%%%%%%%%%%%%%%%%%%%%% initialize_waypoint %%%%%%%%%%%%%%%%%%%%%%%%%%%
% Call the Initialization Function
[waypoint_utm_relative, utm_link_origin] = init_waypoint();

fprintf("\n");
% Display the results of initialization
disp('Waypoint UTM Relative:');
disp(waypoint_utm_relative);

%disp(waypoint_utm_relative(2,2));

%%%%%%%%%%%%%%%%%%%%%%%%%%% TF_utm_map %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert waypoint_utm_relative to a 2D array
waypoint_utm_relative_2d = reshape(waypoint_utm_relative, [], 2);

% Initialize waypoint_map_relative
waypoint_map_relative = zeros(size(waypoint_utm_relative_2d));

% Perform the transformation for each row
for i = 1:size(waypoint_utm_relative_2d, 1)
    waypoint_map_relative(i, :) = TF_utm_map(waypoint_utm_relative_2d(i, :), utm_link_origin);
end

disp('Waypoint Map Relative:');
disp(waypoint_map_relative);

disp('UTM Link Origin:');
disp(utm_link_origin);

%%%%%%%%%%%%%%%%%%%%% calculate_waypoint_angle_utm %%%%%%%%%%%%%%%%%%%%%%%%
waypoint_angle_utm = calculate_waypoint_angle_utm(waypoint_utm_relative);

%%%%%%%%%%%%%%%%%%%%% calculate_waypoint_angle_map %%%%%%%%%%%%%%%%%%%%%%%%
waypoint_angle_map = calculate_waypoint_angle_map(waypoint_map_relative);

%%%%%%%%%%%%%%%%%%%%%% calculate_waypoint_distance %%%%%%%%%%%%%%%%%%%%%%%%
waypoint_distance = calculate_waypoint_distance(waypoint_utm_relative);

%%%%%%%%%%%%%%%%%%%% calculate_waypoint_equation_map %%%%%%%%%%%%%%%%%%%%%%
waypoint_line_equation_map = calculate_waypoint_equation_map(waypoint_map_relative);

%%%%%%%%%%%%%%%%%%%% calculate_waypoint_equation_utm %%%%%%%%%%%%%%%%%%%%%%
waypoint_line_equation_utm = calculate_waypoint_equation_utm(waypoint_utm_relative);

%%%%%%%%%%%%%%%%%%%%%%%%%%% Additional Code %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
test=Point2D;
test.x=1.0;
test.y=1.0;

base_link_origin = Pose2D;
base_link_origin.x = 0.0;
base_link_origin.y = 0.0;
base_link_origin.yaw = 45;

heading_angle_error = waypoint_angle_map(current_waypoint_id) - base_link_origin.yaw;

cross_track_error_utm = calculate_distance_point_line_equation_utm(test, current_waypoint_id, waypoint_line_equation_utm);
cross_track_error_map = calculate_distance_point_line_equation_map(test, current_waypoint_id, waypoint_line_equation_map);
cross_track_error_angle = cross_track_error_map * ctr_factor;

target_steering_angle = heading_angle_error + cross_track_error_angle;

fprintf("distance(utm) = %6.3f\n", cross_track_error_utm);
fprintf("distance(map) = %6.3f\n", cross_track_error_map);
fprintf("heading_angle_error , cross_track_error_angle : %6.3f , %6.3f \n", heading_angle_error, cross_track_error_angle);
fprintf("steering_angle = %6.3f \n", target_steering_angle);

waypoint_arrival_distance = calculate_waypoint_arrival_distance(base_link_origin, current_waypoint_id, waypoint_map_relative, waypoint_angle_map);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_coords = waypoint_utm_relative(:, 1);
y_coords = waypoint_utm_relative(:, 2);

figure;
hold on;
plot(x_coords, y_coords, 'o', 'MarkerSize', 5, 'Color', 'g');
plot(x_coords, y_coords, 'g-');
xlabel('X축');
ylabel('Y축');
title('Waypoint UTM(r) 좌표 그래프');
grid on;

