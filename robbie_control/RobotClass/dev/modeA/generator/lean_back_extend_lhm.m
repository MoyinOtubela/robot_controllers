function [stage_1, stage_2] = lean_back_extend_lhm(x0, resolution, resolution2, obstacle_height)
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
	obj.K2 = 1;
	obj.K3 = 10;
	obj.K4 = 1000;
	obj.K5 = 0;
	% obj.K1 = 1;
	% obj.K2 = 100;
	% obj.K3 = 5;
	% obj.K4 = 20;

	problem.options = optimoptions('fmincon');

	problem.options.MaxFunctionEvaluations = 30000;
	problem.objective = @obj.solve;
	problem.lb = obj.lb;
	problem.ub = obj.ub;
	problem.solver = 'fmincon';

	theta = obj.x0;

	height_limit = 0.3;


	% -0.3505 = 0.8 stab_angle

	stab_trajectory = linspace(0, -0.3, resolution);
	% problem.ub(4) = 0.1
	% problem.lb(4) = 0.1
	% Extend lhm fully
    problem.ub(4) = 2;
    problem.lb(4) = 2;

    problem.ub(2) = 0;
    problem.lb(2) = 0;
    hip_angle_trajectory = linspace(pi/2.5, pi/2, resolution);
    % hip_angle_trajectory = [hip_angle_trajectory, linspace(pi/2.5, pi/2, resolution/2)];

	for i = 1:1:resolution
	  obj.hip_limit = hip_angle_trajectory(i);
	  problem.ub(1) = stab_trajectory(i);
	  problem.lb(1) = stab_trajectory(i);
	  theta = obj.run(problem, theta, height_limit);
	  fprintf('theta(4) = %g\n', theta(4))
	  positions(i, 1) = i;
	  positions(i, 2:9) = convert_to_robot_output(theta);
	  fprintf('pos(4) = %g\n', positions(i, 5))
	  fprintf('Table 1 is %g percent complete\n', (i/resolution)*100)
	  obj.refresh;
	end

	theta_temp = theta;

	% pause(1);

% 	set lhm
	% problem.options.FunctionTolerance = 1e-100;
	% Extend lhm fully
    problem.ub(4) = 1;
    problem.lb(4) = 1;

%   constrain knee to prevent standing
    % problem.ub(2) = 0;
    % problem.lb(2) = 0;

    % stab_trajectory = linspace(theta(1), -0.5389, resolution2);

	stab_height_trajectory = linspace(0, obstacle_height, resolution2);

	positions2 = zeros(resolution2, 9);

	obj.K1 = 1;
	obj.K2 = 100;
	obj.K3 = 1;
	obj.K4 = 10;
	obj.K5 = 0;

	height_limit = 0.4;

    % problem.ub(2) = 0;
    % problem.lb(2) = 0;


	problem.objective = @obj.solve_stab_lhm;

	problem.ub = obj.ub;
	problem.lb = obj.lb;

    problem.ub(4) = 1;
    problem.lb(4) = 1;

    % problem.ub(2) = 0;
    % problem.lb(2) = 0;

    iter=1;

	% for i = 1:1:resolution2
	%   % obj.K2 = ssm_weight_trajectory(i);
	%   obj.stab_height = stab_height_trajectory(i);
	%   theta = obj.run(problem, theta, height_limit);
	%   positions2(i, 1) = i;
	%   positions2(i, 2:9) = obj.convert_to_robot_output(theta);
	%   fprintf('Table 2 is %g percent complete\n', (i/resolution2)*100)
	%   fprintf('pos(4) = %g\n', positions2(i, 5))
	%   obj.refresh;
	% end


	stage_1 = positions;

	% stage_2 = positions2;


	obj.K5 = 0.0;

	% obj.stab_height = obstacle_height+0.02;
	% theta = obj.run(problem, theta, height_limit);
	% obj.refresh;
	% positions3 = zeros(1, 9);
	% positions3(1,1) = 0;
	% positions3(1,2:9) = obj.convert_to_robot_output(theta);

	obj.stab_height = 0.08;
	theta = obj.run(problem, theta, height_limit);


	x1 = linspace(theta_temp(1), theta(1), resolution);
	x2 = linspace(theta_temp(2), theta(2), resolution);
	x3 = linspace(theta_temp(3), theta(3), resolution);
	% x4 = linspace(x0(4), abs(obj.lhm_position), 20);
	x4 = linspace(1, 1, 20);
	% x5 = linspace(x0(5), theta_begin(5), 20);
	% x6 = linspace(x0(6), theta_begin(6), 20);
	x7 = linspace(theta_temp(7), theta(7), resolution);
	x8 = linspace(theta_temp(8), theta(8), resolution);
	obj.stab_height = 0;
	stab_height_trajectory = linspace(0,0.082, resolution*0.2)
	iter = 1;
	trigger = resolution*0.8;
	obj.K1 = 1;
	obj.K2 = 1;
	obj.K3 = 0;
	obj.K4 = 0;
	for i = 1:1:resolution
		if i>trigger
			obj.stab_height = stab_height_trajectory(iter);
			iter = iter + 1;
		end
		problem.lb = [x1(i), x2(i), x3(i), x4(i), -pi, -pi, x7(i), x8(i)];
		problem.ub = [x1(i), x2(i), x3(i), x4(i), pi, pi, x7(i), x8(i)];
		theta = obj.run(problem, theta, 0.35);
		disp(theta)
		% obj.configure([x1(i), x2(i), x3(i), x4(i), x5(i), x6(i), x7(i), x8(i)]);
		% theta(4) = obj.lhm_position;
		obj.refresh();
		positions3(i, 1) = i;
		positions3(i, 2:9) = obj.convert_to_robot_output(theta);
		fprintf('lhm command = %g\n',positions3(i, 5))
		fprintf('Transition Table is %g percent complete\n', (i/resolution)*100)
	end
	

	stage_2 = positions3;
	% stage_3 = [];

	fprintf('theta(4) = %g\n', theta(4))

	% obj.stab_height = 0.2;
	% theta = obj.run(problem, theta, height_limit);

	% stage_3 = [0, stage_2(resolution2,2:9)];
	% stage_3 = [0, stage_2(resolution2,2:9)];
	% stage_3(1, 2) = 0; 

	% stage_3 = [0 0.5 0.1 0.25 -0.15 -0.7924 -0.7924 0 0];

	fprintf('Table is complete')
end

