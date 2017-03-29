close all; clear all;
x0 = [0, 0, 0, 0, 0, 0, 0, 0];
resolution = 20;
resolution2 = 5;
obstacle_height = 0.08;
[stage_1, stage_2] = lean_back_extend_lhm(x0, resolution, resolution2, obstacle_height);

directory = '../trajectories/modeC/'
% save(strcat(directory, 'lbel_waypoints'), 'stage_1', 'stage_2', 'stage_3')
save(strcat(directory, 'lbel_waypoints'), 'stage_1', 'stage_2')

% save(strcat(directory, 'lbel_waypoints'), 'waypoints')


