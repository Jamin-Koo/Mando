function [map_point2d] = TF_utm_map(utm_point2d, utm_link_origin)

Rotation_matrix_axis = set_rotation_matrix_axis(-utm_link_origin.yaw);

map_point2d = zeros(1, 2);

map_point2d(1) = utm_point2d(1) * Rotation_matrix_axis(1, 1) + utm_point2d(2) * Rotation_matrix_axis(1, 2);
map_point2d(2) = utm_point2d(1) * Rotation_matrix_axis(2, 1) + utm_point2d(2) * Rotation_matrix_axis(2, 2);

%disp(map_point2d);

end
