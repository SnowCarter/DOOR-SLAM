function [measurements, edges_id] = generateOutliers(robot_poses, robot1, robot2, node_offset_robot1, node_offset_robot2, number_of_separators, trajectory_size, information_matrix, use_rotation, sigma_R, sigma_t)

measurements.between(1).R = eye(3);
measurements.between(1).t = [0; 0; 0];
measurements.between(1).Info = information_matrix;
edges_id = uint64([]);
sensor_radius = 2;
for i=1:number_of_separators
    robot1_id = ceil(rand*(trajectory_size-1));
    robot2_id = ceil(rand*(trajectory_size-1));
    
    [gt_separator.t, gt_separator.R] = poseSubNoisy3D(robot_poses{robot1}(robot1_id), robot_poses{robot2}(robot2_id), 0, 0);

    separator_offset.t = [0; 0; 0];
    for d = 1:3
        t_offset = rand*sensor_radius;
        while t_offset < 5*sigma_t
            t_offset = rand*sensor_radius;
        end
        separator_offset.t(d) = t_offset;
    end
    if use_rotation
        r_offsets = [0; 0; 0];
        for d = 1:3
            r_offset = rand*360;
            while r_offset < 5*(sigma_R/pi)*180
                r_offset = rand*360;
            end
            r_offsets(d) = r_offset;
        end
        separator_offset.R = rot_mat_x(r_offsets(1))*rot_mat_y(r_offsets(2))*rot_mat_z(r_offsets(3));
    else
        separator_offset.R = eye(3);
    end

    measurements.between(i).R = gt_separator.R * separator_offset.R;
    measurements.between(i).t = gt_separator.t + separator_offset.t;
    measurements.between(i).Info = information_matrix;
    edges_id(end+1,:) = [node_offset_robot1+robot1_id, node_offset_robot2+robot2_id];
end

end

