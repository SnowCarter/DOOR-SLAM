function [matrix3] = rot_mat_x(angle_degre)

    angle_radian = angle_degre/180*pi;
    matrix4 = createRotationOx(angle_radian);
    matrix3 = matrix4(1:3,1:3);

end