close all; clear all;
x0 = [0, 0, 0, 0, 0, 0, 0, 0];
resolution = 20;
resolution2 = 20;
waypoints = lean_back_extend_lhm(x0, resolution, resolution2);

directory = '../trajectories/modeC'

save(strcat(directory, '40_waypoints'), 'waypoints')


