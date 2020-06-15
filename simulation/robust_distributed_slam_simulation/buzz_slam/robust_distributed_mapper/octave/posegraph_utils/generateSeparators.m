function [measurements, edges_id] = generateSeparators(robot_poses, robot1, robot2, node_offset_robot1, node_offset_robot2, number_of_separators, trajectory_size, sigma_R, sigma_t, information_matrix)

measurements.between(1).R = eye(3);
measurements.between(1).t = [0; 0; 0];
measurements.between(1).Info = information_matrix;
edges_id = uint64([]);
for i=1:number_of_separators
    robot1_id = ceil(rand*(trajectory_size-1));
    robot2_id = ceil(rand*(trajectory_size-1));
    [separator.t, separator.R] = poseSubNoisy3D(robot_poses{robot1}(robot1_id), robot_poses{robot2}(robot2_id), sigma_t, sigma_R);
    measurements.between(i).R = separator.R;
    measurements.between(i).t = separator.t;
    measurements.between(i).Info = information_matrix;
    edges_id(end+1,:) = [node_offset_robot1+robot1_id, node_offset_robot2+robot2_id];
end

end

