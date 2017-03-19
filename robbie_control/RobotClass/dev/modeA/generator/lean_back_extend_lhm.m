function [stage_1, stage_2, stage_3] = lean_back_extend_lhm(x0, resolution, resolution2, obstacle_height)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    clc; close all;
	addpath('../../../');
	addpath('../../');

	positions = zeros(resolution, 9);

	obj = LookupTableGenerator();
	obj.set_head_mass(1e-6);
	% obj.set_head_mass(0);
	obj.configure(x0);
	obj.refresh();
	obj.stab_height = 0.0;

	obj.K1 = 1;
	obj.K2 = 2;
	obj.K3 = 10;
	obj.K4 = 1;
	% obj.K1 = 1;
	% obj.K2 = 100;
	% obj.K3 = 5;
	% obj.K4 = 20;

	problem.options = optimoptions('fmincon','StepTolerance',1e-100);
	% problem.options = optimoptions('fmincon','FunctionTolerance',1e-100);
	% problem.options = optimoptions('fmincon','UseParallel', 0);

	% problem.options = optimoptions('fmincon','Display','iter');

	problem.options.MaxFunctionEvaluations = 30000;
	problem.objective = @obj.solve;
	problem.lb = obj.lb;
	problem.ub = obj.ub;
	problem.solver = 'fmincon';

	theta = obj.x0;

	height_limit = 0.35;

	obj.com_height_limit = height_limit;

	% -0.3505 = 0.8 stab_angle

	stab_trajectory = linspace(0, -0.3, resolution);
	% height_trajectory = linspace(0.5, 0.38, resolution);
	height_trajectory = linspace(0.375, height_limit, resolution/2);
	% height_trajectory = [height_trajectory, linspace(height_limit, 0.375, resolution/2)];
	% problem.ub(4) = 0.1
	% problem.lb(4) = 0.1
	% Extend lhm fully
    problem.ub(4) = 1;
    problem.lb(4) = 1;

	for i = 1:1:resolution
	  problem.ub(1) = stab_trajectory(i);
	  problem.lb(1) = stab_trajectory(i);

	  theta = obj.run(problem, theta, height_limit);
	  fprintf('theta(4) = %g\n', theta(4))
	  positions(i, 1) = i;
	  positions(i, 2:9) = convert_to_robot_output(theta);
	  fprintf('Table 1 is %g percent complete\n', (i/resolution)*100)
	  obj.refresh;
	  % fprintf('hip angle %f\n', obj.hip_monitor*180/pi);
	end

	% pause(1);

% 	set lhm
	% problem.options.FunctionTolerance = 1e-100;
	% Extend lhm fully
    problem.ub(4) = 1;
    problem.lb(4) = 1;

%   constrain knee to prevent standing
    % problem.ub(2) = 0;
    % problem.lb(2) = 0;

    stab_trajectory = linspace(theta(1), -0.5389, resolution2);

	stab_height_trajectory = linspace(0, 0.2, resolution2*0.2);

	positions2 = zeros(resolution2, 9);

	% obj.K1 = 1;
	% obj.K2 = 100;
	% obj.K3 = 300;
	% obj.K4 = 20;
	obj.K1 = 1;
	obj.K2 = 700;
	obj.K3 = 100;
	obj.K4 = 20;

	height_limit = 0.3;

	index = 1;
	for i = 1:1:resolution2
	  if i>resolution2*0.8
		problem.ub(2) = 0.25;
		problem.lb(2) = 0;
		obj.hip_limit = pi/1.5;
	  	% obj.stab_height = stab_height_trajectory(i-resolution2*0.8);
	  end
	  problem.ub(1) = stab_trajectory(i);
	  problem.lb(1) = stab_trajectory(i);
	  theta = obj.run(problem, theta, height_limit);
	  positions2(i, 1) = i;
	  positions2(i, 2:9) = obj.convert_to_robot_output(theta);
	  fprintf('Table 2 is %g percent complete\n', (i/resolution2)*100)
	  obj.refresh;
	end


	stage_1 = positions;

	stage_2 = positions2;

	obj.stab_height = obstacle_height+0.02;
	theta = obj.run(problem, theta, height_limit);
	obj.refresh;
	positions3 = zeros(1, 9);
	positions3(1,1) = 0;
	positions3(1,2:9) = obj.convert_to_robot_output(theta);
	
	stage_3 = positions3;

	fprintf('theta(4) = %g\n', theta(4))

	% obj.stab_height = 0.2;
	% theta = obj.run(problem, theta, height_limit);

	% stage_3 = [0, stage_2(resolution2,2:9)];
	% stage_3 = [0, stage_2(resolution2,2:9)];
	% stage_3(1, 2) = 0; 

	% stage_3 = [0 0.5 0.1 0.25 -0.15 -0.7924 -0.7924 0 0];

	fprintf('Table is complete')
end

