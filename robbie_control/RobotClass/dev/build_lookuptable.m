function [positions] = build_lookuptable(x0, resolution)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    clc;
	addpath('../')
	positions = zeros(resolution, 9);

	obj = LookupTableGenerator();
	obj.configure(x0);
	obj.refresh();
	obj.K1 = 1000;
	obj.K2 = 1000;
	obj.K3 = 1;

	% obj.mass(15) = 100;

	problem.options = optimoptions('fmincon');
	problem.options.MaxFunctionEvaluations = 5000;
	problem.objective = @obj.solve;
	problem.x0 = obj.x0;
	problem.lb = obj.lb;
	problem.ub = obj.ub;
	problem.solver = 'fmincon';

	% z = linspace(0.38, 0.5878, resolution);
	% z = linspace(0.38, 0.8, resolution);
	z = linspace(0.38, 1, resolution);
	% z = linspace(0.6, 0.8, resolution);
	index = 1;
	for i = z
	  theta = obj.run(problem, obj.x0, i);
	  positions(index, 1) = obj.height;
	  positions(index, 2:9) = convert_to_robot_output(theta);
	  fprintf('Table is %g percent complete\n', (index/resolution)*100)
	  index = index + 1;
	  obj.refresh;
	end
	  fprintf('Table is complete', (index/resolution)*100)
end

