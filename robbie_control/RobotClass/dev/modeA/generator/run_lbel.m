close all; clear all;
x0 = [0, 0, 0, 0, 0, 0, 0, 0];
resolution = 10;
resolution2 = 5;
waypoints = lean_back_extend_lhm(x0, resolution, resolution2);

directory = '../trajectories/modeC/'

save(strcat(directory, 'lbel_waypoints'), 'waypoints')


