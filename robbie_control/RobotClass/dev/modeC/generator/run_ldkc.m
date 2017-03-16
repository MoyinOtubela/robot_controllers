close all; clear all;
% x0 = [-0.3, 0.1, 0.25, 0.15, -0.7924, -0.7924, 0, 0];

resolution = 10;
resolution2 = 5;
addpath('../../../');
addpath('../../');

x0 = [-0.5389, 0, 0.0106, 0.168529, -1.3848, -1.3848, 0, 0];
obj = LookupTableGenerator();
obj.stab_height = 0.2;
% % % obj.shank_rotation = -0.17295;
% % % x0 = [0, 0, 0, 0, 0, 0, 0, 0];

obj.configure(x0);
obj.refresh();
[stage_1, stage_2, stage_3] = lhm_down_knee_constant(x0, resolution, resolution2);

% x0 = [-0.5389, 0.0948, -0.0163, 0.3, -0.7909, -0.7909, 0, 0];
% x0 = [-0.5389, 0.7, -0.0163, 0.3, -0.7909, -0.7909, 0, 0];
% addpath('../../../');
% addpath('../../');

% positions = zeros(resolution, 9);
% -0.5389    0.0948   -0.0163    0.1800   -0.7909   -0.7909         0

%   Column 8

%          0

	% x0 = [1, 0.1, 0.7, 0, -0.7924, -0.7924, 0, 0]


directory = '../trajectories/climb/'

save(strcat(directory, 'ldkc_waypoints'), 'stage_1', 'stage_2', 'stage_3')


