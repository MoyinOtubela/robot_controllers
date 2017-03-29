close all; clear all;
x0 = [0, 0, 0, 0, 0, 0, 0, 0];
resolution = 100;
[stage_1] = height_adjustment(x0, resolution);

directory = '../trajectories/stand/'
save(strcat(directory, 'height_adjustment'), 'stage_1')



