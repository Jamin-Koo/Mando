function Rotation_matrix_axis = set_rotation_matrix_axis(m_angle_degree)
    angle_radian = m_angle_degree / 180 * pi;
    
    Rotation_matrix_axis = [cos(angle_radian), sin(angle_radian); 
                            -sin(angle_radian), cos(angle_radian)];
end