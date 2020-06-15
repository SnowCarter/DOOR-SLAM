function [matrix3] = rot_mat_y(angle_degre)

    angle_radian = angle_degre/180*pi;
    matrix4 = createRotationOy(angle_radian);
    matrix3 = matrix4(1:3,1:3);

end