function [map_point] = TF_utm_map_X(utm_point, rotation_matrix)
    disp(utm_point);
    
    for i = 1:size(utm_point, 1)
        map_point(i,1) = utm_point(i, 1) * rotation_matrix(1, 1) + utm_point(i,2) * rotation_matrix(1, 2);
        map_point(i,2) = utm_point(i, 1) * rotation_matrix(2, 1) + utm_point(i,2) * rotation_matrix(2, 2);
    end

end