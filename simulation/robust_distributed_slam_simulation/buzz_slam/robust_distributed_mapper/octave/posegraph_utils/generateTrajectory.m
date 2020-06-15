function [poses, measurements, edges_id] = generateTrajectory(node_offset, trajectory_size, trajectory_offset, information_matrix, use_rotation, sigma_R, sigma_t)

poses(1).t = [0; 0; 0] + trajectory_offset(1:3);
poses(1).R = rot_mat_x(trajectory_offset(4))*rot_mat_y(trajectory_offset(5))*rot_mat_z(trajectory_offset(6));
measurements.between(1).R = eye(3);
measurements.between(1).t = [0; 0; 0];
measurements.between(1).Info = information_matrix;
edges_id = uint64([]);
t_speed = 1;
for i=1:trajectory_size
    if use_rotation
        offset.R = rot_mat_x(360*rand)*rot_mat_y(360*rand)*rot_mat_z(360*rand);
    else
        offset.R = eye(3);
    end
    offset.t = [2*t_speed*rand-t_speed; 2*t_speed*rand-t_speed; 2*t_speed*rand-t_speed];
    offset.t = offset.t ./ norm(offset.t);
    measurements.between(i).R = offset.R;
    measurements.between(i).t = offset.t;
    
    % Nominal noise
    e_rot_vec  = gaussian_noise(sigma_R^2 * eye(3));
    e_trans_vec = gaussian_noise(sigma_t^2 * eye(3));

    % Add noise to measurements 
    if(norm(e_rot_vec)>1e-7)
        e_rot = axis_angle_to_mat([e_rot_vec'/norm(e_rot_vec) norm(e_rot_vec)]);
        measurements.between(i).R = measurements.between(i).R * e_rot; % perturb the rotation measurement
    end
    measurements.between(i).t = measurements.between(i).t + e_trans_vec; 

    measurements.between(i).Info = information_matrix;
    [new_pose.t, new_pose.R] = poseAdd3D(poses(end), offset);
    new_pose.t = new_pose.t;
    poses(i+1) = new_pose;
    edges_id(end+1, :) = [uint64(node_offset+i), uint64(node_offset+i+1)];
end

end

