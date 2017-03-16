function [stage_1, stage_2, stage_3] = lhm_down_knee_constant(x0, resolution, resolution2)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    clc; close all;
	addpath('../../../');
	addpath('../../');

	positions = zeros(resolution, 9);
	positions2 = zeros(resolution2, 9);
	% obj.shank_rotation = -0.17295;

	obj = LookupTableGenerator();
	
	obj.set_head_mass(1e-6);
	obj.configure(x0);

	obj.refresh();

	obj.K1 = 1;
	obj.K2 = 1;
	obj.K3 = 1;
	obj.K4 = 1;

	% problem.options = optimoptions('fmincon','StepTolerance',1e-100, 'FunctionTolerance', 1e-50);
	% problem.options = optimoptions('fmincon','FunctionTolerance',1e-100);
	problem.options = optimoptions('fmincon','UseParallel', 0);

	% problem.options = optimoptions('fmincon','Display','iter');

	problem.options.MaxFunctionEvaluations = 30000;
	problem.objective = @obj.solve;
	problem.lb = obj.lb;
	problem.ub = obj.ub;
	problem.solver = 'fmincon';

	index = 1
	theta = obj.x0;

	height_limit = 0.35;

	height_trajectory = linspace(0.35, 0.7, resolution);
	shank_angle_trajectory = linspace(-0.17295, 0, resolution);
	lhm_trajectory = linspace(0.15, 0.31, resolution);
	obstacle_height = 0.08;
	obj.obstacle_height = obstacle_height+0.006;
	shank_height_trajectory = linspace(0, obj.obstacle_height, resolution);
	obj.stab_height = obstacle_height;

	% stab_height_trajectory = linspace(0.2, obstacle_height, resolution);

	problem.ub(2) = 0;
	problem.lb(2) = 0;
	problem.lb(1) = -0.3;
	problem.ub(1) = 0;

	problem.ub(4) = 1;
	problem.lb(4) = 1;


	for i = 1:1:resolution
	  % obj.stab_height = stab_height_trajectory()
	  % obj.shank_rotation = shank_angle_trajectory(i);
	  obj.shank_height = shank_height_trajectory(i);
	  theta = obj.run(problem, theta, height_limit);
	  % lhm_y = (obj.joint_locations(6, 2) - obj.joint_locations(17, 2))^2;
	  % lhm_z = (obj.joint_locations(6, 3) - obj.joint_locations(17, 3))^2;
	  % theta(4) = sqrt(lhm_y + lhm_z);
	  theta(4) = -obj.lhm_position;
	  positions(index, 1) = i;
	  positions(index, 2:9) = convert_to_robot_output(theta);
	  fprintf('lhm command = %g',theta(4))
	  % fprintf('Table 1 is %g percent complete\n', (index/resolution)*100)
	  index = index + 1;
	  obj.refresh;
	end

	problem.ub = [0 1 0.8 0 0.383972435 0.383972435 0 0];
	problem.lb = [-pi/4 0 -1 0 -0.383972435 -0.383972435 0 0];
	com_height_trajectory = linspace(obstacle_height, obstacle_height+0.5, resolution2);

	% index = 1;
	% lhm_trajectory = linspace(thet, 0.31, resolution);
	obj.K1 = 1;
	obj.K2 = 10000;
	obj.K3 = 100;
	obj.K4 = 1;
	problem.ub(4) = 0;
	problem.lb(4) = 0;


	for i = 1:1:resolution2
	  theta = obj.run(problem, theta, com_height_trajectory(i));
	  positions2(i, 1) = i;
	  % lhm_y = (obj.joint_locations(6, 2) - obj.joint_locations(16, 2))^2;
	  % lhm_z = (obj.joint_locations(6, 3) - obj.joint_locations(16, 3))^2;
	  % theta(4) = sqrt(lhm_y + lhm_z);
  	  theta(4) = -obj.lhm_position;

	  positions2(i, 2:9) = convert_to_robot_output(theta);
	  fprintf('Table 2 is %g percent complete\n', (i/resolution2)*100)
	  obj.refresh;
	end

	% pause(1);

% 	set lhm
	% problem.options.FunctionTolerance = 1e-100;
%     problem.ub(4) = 0.18;
%     problem.lb(4) = 0.18;

% %   constrain knee to prevent standing
%     % problem.ub(2) = 0;
%     % problem.lb(2) = 0;

%     stab_trajectory = linspace(theta(1), -0.5389, resolution2);

% 	stab_height_trajectory = linspace(0, 0.1, resolution2*0.2);

% 	positions2 = zeros(resolution2, 9);

% 	obj.K1 = 1;
% 	obj.K2 = 100;
% 	obj.K3 = 300;
% 	obj.K4 = 20;

% 	height_limit = 0.3;

% 	index = 1;
% 	for i = 1:1:resolution2
% 	  if i>resolution2*0.8
% 		problem.ub(2) = 0.25;
% 		problem.lb(2) = 0;
% 	  	obj.stab_height = stab_height_trajectory(i-resolution2*0.8);
% 	  end
% 	  problem.ub(1) = stab_trajectory(i);
% 	  problem.lb(1) = stab_trajectory(i);
% 	  theta = obj.run(problem, theta, height_limit);
% 	  positions2(index, 1) = i;
% 	  positions2(index, 2:9) = convert_to_robot_output(theta);
% 	  fprintf('Table 2 is %g percent complete\n', (index/resolution2)*100)
% 	  index = index + 1;
% 	  % obj.refresh;
% 	  % fprintf('hip angle %f\n', obj.hip_monitor*180/pi);
% 	end

	stage_1 = positions;

	stage_2 = positions2;

	stage_3 = [0 0 0 0 0 0 0 0 0];

% 	waypoints.stage_3 = [0 0.5 0.1 0.25 -0.15 -0.7924 -0.7924 0 0];
	


	fprintf('Table is complete')
end

