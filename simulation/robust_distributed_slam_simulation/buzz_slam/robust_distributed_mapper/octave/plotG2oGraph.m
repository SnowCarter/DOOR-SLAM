%% Author: Pierre-Yves Lajoie (lajoie.py@gmail.com)

%% Global initialization
clear all
close all
clc

% Add graph utils to path
addpath(genpath('./posegraph_utils'));

% Constant
file_name = '7.g2o';

% Read g2o file
[measurements, edges_id, ~, ~, ~, ~, ~, ~, ~, ~] = readG2oDataset3D(file_name);

poses_2d = [0 0];
previous_pose.t = [0;0;0];
previous_pose.R = eye(3);
number_of_poses = 1
for i = 1:size(measurements.between,2)
    if (abs(edges_id(i,1)-edges_id(i,2)) <= 1)
        [current_pose.t current_pose.R] = poseAdd3D(previous_pose, measurements.between(i)) ;
        previous_pose = current_pose;
        poses_2d(number_of_poses+1, 1:2) = [current_pose.t(1), current_pose.t(2)];
        number_of_poses = number_of_poses + 1;
    end
end

figure
plot(poses_2d(:,1), poses_2d(:,2))
axis equal