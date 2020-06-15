%% Author: Pierre-Yves Lajoie (lajoie.py@gmail.com)

%% Global initialization
clear all
close all
clc

% Add graph utils to path
addpath(genpath('./posegraph_utils'));

% Constant
number_of_robots = 10
dataset_folder = '../../../argos_simulation/log/datasets/';
file_extension = '_optimized.g2o';
colors = {[0.5 0.5 0], [0.5 0 0.5], [0 1 1], [0.5 0 0], [0 1 0], [0 0 1], [0 0 0], [0 0.5 1], [0.5 1 0], [0.5 0 1]};

% Initialize figure
figure
axis equal
hold on

for robot = 0:number_of_robots-1

    file_name = [dataset_folder, num2str(robot), file_extension]
    is_exist = exist(file_name, 'file');
    if (is_exist == 2) 
        % Read g2o file
        [~, ~, poses, ~, ~, ~, ~, ~, ~, ~] = readG2oDataset3D(file_name);

        for i = 1:size(poses,2)
            poses_2d(i, 1:2) = [poses(i).t(1), poses(i).t(2)];
        end

        plot(poses_2d(:,1), poses_2d(:,2), 'Color', colors{robot+1})
    end

end