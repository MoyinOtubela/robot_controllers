close all; clear all;
x0 = [0, 0, 0, 0, 0, 0, 0, 0];
resolution = 10;
resolution2 = 10;
[stage_1, stage_2, stage_3] = lean_back_extend_lhm(x0, resolution, resolution2);

directory = '../trajectories/modeC/'
save(strcat(directory, 'lbel_waypoints'), 'stage_1', 'stage_2', 'stage_3')

% save(strcat(directory, 'lbel_waypoints'), 'waypoints')


