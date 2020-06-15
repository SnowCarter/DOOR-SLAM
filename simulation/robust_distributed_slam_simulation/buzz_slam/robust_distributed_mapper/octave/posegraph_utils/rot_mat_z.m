function [matrix3] = rot_mat_z(angle_degre)

    angle_radian = angle_degre/180*pi;
    matrix4 = createRotationOz(angle_radian);
    matrix3 = matrix4(1:3,1:3);

end