function [stage_1] = height_adjustment(x0, resolution)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    clc; close all;
	addpath('../../../');
	addpath('../../');

	positions = zeros(resolution, 9);

	obj = LookupTableGenerator();
	obj.set_head_mass(1e-6);
	% obj.set_head_mass(0);
	% obj.configure(x0);
	% obj.refresh();
	% obj.stab_height = 0;

	obj.K1 = 1;
	obj.K2 = 10;
	obj.K3 = 100;
	obj.K4 = 1;

	problem.options = optimoptions('fmincon','StepTolerance',1e-100);
	% problem.options = optimoptions('fmincon','FunctionTolerance',1e-100);
	% problem.options = optimoptions('fmincon','UseParallel', 0);

	% problem.options = optimoptions('fmincon','Display','iter');
            % obj.ub = [0 1 0.8 0 0.383972435 0.383972435 0 0];
            % obj.lb = [-pi/4 0 -1 0 -pi -pi 0 0];

	problem.options.MaxFunctionEvaluations = 30000;
	problem.objective = @obj.solve;
	problem.lb = obj.lb;
	problem.ub = obj.ub;
	problem.solver = 'fmincon';

	problem.lb(5) = -pi/2;
	problem.lb(6) = -pi/2;
	problem.ub(5) = 0;
	problem.ub(6) = 0;

	theta = obj.x0;

	% stability_trajectory = linspace(1, 1000, resolution);

	% stab_trajectory = linspace(0, -0.3, resolution);
	% height_trajectory = linspace(0.5, 0.38, resolution);
	height_trajectory = linspace(0.3, 0.6, resolution);
	% Extend lhm fully

	for i = 1:1:resolution
	  % obj.K2 = stability_trajectory(i);
	  theta = obj.run(problem, theta, height_trajectory(i));
	  fprintf('theta(4) = %g\n', theta(4))
	  positions(i, 1) = obj.height;
	  positions(i, 2:9) = convert_to_robot_output(theta);
	  fprintf('Table 1 is %g percent complete\n', (i/resolution)*100)
	  obj.refresh;
	  % fprintf('hip angle %f\n', obj.hip_monitor*180/pi);
	end



	stage_1 = positions;
	fprintf('Table is complete')
end

