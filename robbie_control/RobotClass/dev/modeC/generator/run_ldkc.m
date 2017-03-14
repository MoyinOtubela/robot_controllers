close all; clear all;
% x0 = [-0.3, 0, 0, 0, 0, 0, 0, 0];
% x0 = [-0.3, 0.1, 0.25, 0.15, -0.7924, -0.7924, 0, 0];

resolution = 10;
resolution2 = 5;

x0 = [-0.5389, 0.0948, -0.0163, 0.15, -0.7909, -0.7909, 0, 0];
waypoints = lhm_down_knee_constant(x0, resolution, resolution2);

% x0 = [-0.5389, 0.0948, -0.0163, 0.3, -0.7909, -0.7909, 0, 0];
% x0 = [-0.5389, 0.7, -0.0163, 0.3, -0.7909, -0.7909, 0, 0];
% addpath('../../../');
% addpath('../../');

% positions = zeros(resolution, 9);
% -0.5389    0.0948   -0.0163    0.1800   -0.7909   -0.7909         0

%   Column 8

%          0

	% x0 = [1, 0.1, 0.7, 0, -0.7924, -0.7924, 0, 0]


% directory = '../trajectories/climb/'

% save(strcat(directory, 'lbel_waypoints'), 'waypoints')


