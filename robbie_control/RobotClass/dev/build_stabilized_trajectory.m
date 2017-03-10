function [positions] = build_stabilized_trajectory(x0, resolution)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    clc; close all;
	addpath('../')
	positions = zeros(resolution, 9);

	obj = LookupTableGenerator();
	obj.set_head_mass(1e-6);
	% obj.set_head_mass(0);
	obj.configure(x0);
	obj.refresh();
	% obj.K1 = 1;
	% obj.K2 = 1000;
	% obj.K3 = 1;
	obj.K1 = 1;
	obj.K2 = 150;
	obj.K3 = 200;
	obj.K4 = 6;

	% problem.options = optimoptions('fmincon','StepTolerance',1e-100);
	% problem.options = optimoptions('fmincon','FunctionTolerance',1e-100);
	problem.options = optimoptions('fmincon');
	% problem.options = optimoptions('fmincon','Display','iter');
	problem.options.MaxFunctionEvaluations = 30000;
	problem.objective = @obj.solve;
	problem.lb = obj.lb;
	problem.ub = obj.ub;
	problem.solver = 'fmincon';


	% z = linspace(0.38, 0.5878, resolution);
	% z = linspace(0.4, 0.6, resolution);
	% z = linspace(0.5, 1, resolution);
	% z = linspace(0.38, 0.8, resolution);
	% z = linspace(0.38, 1, resolution);



	index = 1;
	theta = obj.x0;

	height_limit = 0.35;

	obj.com_height_limit = height_limit;

	% -0.3505 = 0.8 stab_angle

	stab_trajectory = linspace(0, -0.3505, resolution);
	% height_trajectory = linspace(0.5, 0.38, resolution);
	height_trajectory = linspace(0.375, 0.4, resolution/2);
	height_trajectory = [height_trajectory, linspace(0.4, 0.375, resolution/2)];
	% problem.ub(4) = 0.1
	% problem.lb(4) = 0.1

	for i = 1:1:resolution
	  problem.ub(1) = stab_trajectory(i);
	  problem.lb(1) = stab_trajectory(i);
	  % theta = obj.run(problem, theta, height_limit);
	  theta = obj.run(problem, theta, height_trajectory(i));
	  positions(index, 1) = obj.height;
	  positions(index, 2:9) = convert_to_robot_output(theta);
	  fprintf('Table is %g percent complete\n', (index/resolution)*100)
	  index = index + 1;
	  obj.refresh;
	  fprintf('hip angle %f\n', obj.hip_monitor*180/pi);
	end


    problem.ub(4) = 0.15;
    problem.lb(4) = 0.15;

    % theta = obj.run(problem, theta, height_trajectory(index-1));
    theta = obj.run(problem, theta, height_limit);

	positions(index, 1) = obj.height;
	positions(index, 2:9) = convert_to_robot_output(theta);
	obj.refresh;
	fprintf('Table is complete', (index/resolution)*100)
end

