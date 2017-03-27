close all; clear all;
% x0 = [-0.3, 0.1, 0.25, 0.15, -0.7924, -0.7924, 0, 0];

resolution = 20;
resolution2 = 20;
addpath('../../../');
addpath('../../');
% x0 = [-0.5389, 0, 0.0126, 0.168529, -1.48528, -1.48528, 0, 0];

% x0 = [-0.5389, 0, 0.0126, 0.168529, -1.48528, -1.48528, 0, 0];

x0 = [-0.7854, 0.3361, 0.0909, 0.226009, 0.3839, 0.3839, 0, 0];


obj = LookupTableGenerator();

obstacle_height = 0;

% obj.configure(x0);

% obj.refresh();

[stage_1, stage_2, stage_3, stage_4] = stab_down_shank_up(x0, resolution, resolution2, obstacle_height);


directory = '../trajectories/modeB/'

% save(strcat(directory, 'sdsu_waypoints'), 'stage_1', 'stage_2')
% save(strcat(directory, 'sdsu_waypoints'), 'stage_1', 'stage_2','stage_3')
save(strcat(directory, 'sdsu_waypoints'), 'stage_1', 'stage_2', 'stage_3','stage_4')


