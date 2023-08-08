function Rotation_matrix_point = set_rotation_matrix_point(m_angle_degree)
    angle_radian = m_angle_degree / 180 * pi;

    Rotation_matrix_point = [cos(angle_radian), -sin(angle_radian);
                             sin(angle_radian), cos(angle_radian)];
end