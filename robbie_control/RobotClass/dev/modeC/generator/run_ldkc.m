close all; clear all;
% x0 = [-0.3, 0.1, 0.25, 0.15, -0.7924, -0.7924, 0, 0];

resolution = 10;
resolution2 = 20;
addpath('../../../');
addpath('../../');

x0 = [-0.5389, 0, 0.0126, 0.168529, -1.48528, -1.48528, 0, 0];
obj = LookupTableGenerator();
% obj.stab_height = 0.08;
obstacle_height = 0.08;
% % % obj.shank_rotation = -0.17295;
% % % x0 = [0, 0, 0, 0, 0, 0, 0, 0];

obj.configure(x0);
obj.refresh();

[stage_1, stage_2, stage_3] = lhm_down_knee_constant(x0, resolution, resolution2, obstacle_height);


directory = '../trajectories/climb/'

save(strcat(directory, 'ldkc_waypoints'), 'stage_1', 'stage_2', 'stage_3')


