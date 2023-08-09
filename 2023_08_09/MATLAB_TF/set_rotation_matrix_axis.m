function [Rotation_matrix_axis] = set_rotation_matrix_axis(m_angle_degree)

    angle_radian = deg2rad(m_angle_degree);
    
    Rotation_matrix_axis = zeros(2);  % 2x2 행렬 생성

    Rotation_matrix_axis(1, 1) = cos(angle_radian);
    Rotation_matrix_axis(1, 2) = sin(angle_radian);
    Rotation_matrix_axis(2, 1) = -sin(angle_radian);
    Rotation_matrix_axis(2, 2) = cos(angle_radian);

    %disp(Rotation_matrix_axis);

end