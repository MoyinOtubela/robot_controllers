function [positions] = build_lookuptable(x0, resolution)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    clc; close all;
	addpath('../')
	positions = zeros(resolution, 9);

	obj = LookupTableGenerator();
	% obj.set_head_mass(1e-6);
	obj.set_head_mass(0);
	obj.configure(x0);
	obj.refresh();
	% obj.K1 = 1;
	% obj.K2 = 1000;
	% obj.K3 = 1;
	obj.K1 = 1;
	obj.K2 = 5;
	obj.K3 = 10;

	% problem.options = optimoptions('fmincon','StepTolerance',1e-100);
	problem.options = optimoptions('fmincon','FunctionTolerance',1e-100);
	% problem.options = optimoptions('fmincon','Display','iter');
	problem.options.MaxFunctionEvaluations = 30000;
	problem.objective = @obj.solve;
	problem.lb = obj.lb;
	problem.ub = obj.ub;
	problem.solver = 'fmincon';


	% z = linspace(0.38, 0.5878, resolution);
	z = linspace(0.4, 0.6, resolution);
	% z = linspace(0.5, 1, resolution);
	% z = linspace(0.38, 0.8, resolution);
	% z = linspace(0.38, 1, resolution);

	index = 1;
	% theta = obj.run(problem, x0, z(1));
	% theta = obj.run(problem, x0, 0.45);
	theta = obj.x0;

	

	for i = z
	  theta = obj.run(problem, theta, i);
	  positions(index, 1) = obj.height;
	  positions(index, 2:9) = convert_to_robot_output(theta);
	  fprintf('Table is %g percent complete\n', (index/resolution)*100)
	  index = index + 1;
	  obj.refresh;
	  fprintf('hip angle %f\n', obj.hip_monitor*180/pi);
	end
	fprintf('Table is complete', (index/resolution)*100)
end

