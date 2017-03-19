function [stage_1, stage_2, stage_3, stage_4] = stab_down_shank_up(x0, resolution, resolution2, obstacle_height)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    clc; close all;
	addpath('../../../');
	addpath('../../');

	positions = zeros(resolution, 9);

	obj = LookupTableGenerator();
	obj.set_head_mass(1e-6);

	obj.K1 = 1;
	obj.K2 = 1e-6;
	obj.K3 = 1e2;
	obj.K4 = 10;

	problem.options = optimoptions('fmincon','StepTolerance',1e-100);
	% problem.options = optimoptions('fmincon','FunctionTolerance',1e-100);
	% problem.options = optimoptions('fmincon','UseParallel', 0);
	% problem.options = optimoptions('fmincon','Display','iter');

	problem.options.MaxFunctionEvaluations = 30000;
	problem.objective = @obj.solve;
	problem.lb = obj.lb;
	problem.ub = obj.ub;
	problem.solver = 'fmincon';

	height_limit = 0.35;

	theta = obj.x0;

    problem.ub(4) = 1;
    problem.lb(4) = 1;

	positions = zeros(1, 9);
	positions2 = zeros(resolution, 9);
	positions3 = zeros(resolution2, 9);
	positions4 = zeros(resolution2, 9);


	obj.stab_height = obstacle_height;
	theta = obj.run(problem, theta, height_limit);
	obj.refresh;
	positions = zeros(1, 9);
	positions(1,1) = 0;
	positions(1,2:9) = obj.convert_to_robot_output(theta);

	% obj.com_height_limit = height_limit;

	% stab_trajectory = linspace(0.2, 0, resolution);
	shank_height_trajectory = linspace(0, 0.03, resolution);



	% -0.3505 = 0.8 stab_angle
	obj.K1 = 1;
	obj.K2 = 0.7e2;
	obj.K3 = 10;
	obj.K4 = 10;

	% Extend lhm fully
    problem.ub(4) = 1;
    problem.lb(4) = 1;
	problem.ub(1) = -0.5;
	problem.lb(1) = -0.5;

	for i = 1:1:resolution
	  obj.shank_height = shank_height_trajectory(i);
	  theta = obj.run(problem, theta, height_limit);
	  theta(4) = -abs(theta(4));
	  fprintf('theta(4) = %g\n', theta(4))
	  positions2(i, 1) = i;
	  positions2(i, 2:9) = obj.convert_to_robot_output(theta);
	  fprintf('Table 1 is %g percent complete\n', (i/resolution)*100)
	  obj.refresh;
	  % fprintf('hip angle %f\n', obj.hip_monitor*180/pi);
	end

	shank_height_trajectory = linspace(0.03, 0, resolution2);


	for i = 1:1:resolution2
	  obj.shank_height = shank_height_trajectory(i);
	  theta = obj.run(problem, theta, height_limit);
	  fprintf('theta(4) = %g\n', theta(4))
	  theta(4) = -abs(theta(4));
	  positions3(i, 1) = i;
	  positions3(i, 2:9) = obj.convert_to_robot_output(theta);
	  fprintf('Table 2 is %g percent complete\n', (i/resolution2)*100)
	  obj.refresh;
	  % fprintf('hip angle %f\n', obj.hip_monitor*180/pi);
	end

	obj.K1 = 1;
	obj.K2 = 1e1;
	obj.K3 = 1e2;
	obj.K4 = 1e-2;


	height_limit = 0.375

	problem.ub = [0 1 0.8 0 0.383972435 0.383972435 0 0];
	problem.lb = [-pi/4 0 -1 0 -pi -pi 0 0];

    problem.ub(4) = 0.3;
    problem.lb(4) = 0.3;

	shank_trajectory = linspace(-0.5, 0, resolution2);
	lhm_trajectory = linspace(-theta(4)-0.05, 0, resolution2);
	height_trajectory = linspace(0.4, 0.3, resolution2);


	count = 1;

	stop_condition = resolution2*0.3;

	for i = 1:1:resolution2

	  if i > stop_condition
	  	obj.K4 = 10;
	  	obj.K2 = 1e3;
	    problem.lb(4) = 0;
	    problem.ub(4) = 0;
	  end
	  % count = count + 1;
	  problem.lb(1) = shank_trajectory(i);
	  problem.ub(1) = shank_trajectory(i);
	  theta = obj.run(problem, theta, height_trajectory(i));
	  theta(4) = -abs(theta(4));
	  fprintf('theta(4) = %g\n', theta(4))
	  positions4(i, 1) = i;
	  positions4(i, 2:9) = obj.convert_to_robot_output(theta);
	  fprintf('Table 3 is %g percent complete\n', (i/resolution2)*100)
	  obj.refresh;
	  % fprintf('hip angle %f\n', obj.hip_monitor*180/pi);
	end

	stage_1 = positions;

	stage_2 = positions2;

	stage_3 = positions3;

	stage_4 = positions4;

	fprintf('Table is complete')
end

