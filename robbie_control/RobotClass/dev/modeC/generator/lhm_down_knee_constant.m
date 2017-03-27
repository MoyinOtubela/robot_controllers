function [stage_1, stage_2, stage_3] = lhm_down_knee_constant(x0, resolution, resolution2, obstacle_height)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    clc; close all;
	addpath('../../../');
	addpath('../../');

	obj = LookupTableGenerator();

 %    x0 = [-0.7854, 0.3361, 0.0909, 0.226007, 0.3839, 0.3839, 0, 0]
	% xT = [-0.135, 0, -0.01, 0.186426, 0, 0, 0, 0]
	% x1 = linspace(x0(1), xT(1), 20);
	% x2 = linspace(x0(2), xT(2), 20);
	% x3 = linspace(x0(3), xT(3), 20);
	% x4 = linspace(x0(4), xT(4), 20);
	% x5 = linspace(x0(5), xT(5), 20);
	% x6 = linspace(x0(6), xT(6), 20);
	% x7 = linspace(x0(7), xT(7), 20);
	% x8 = linspace(x0(8), xT(8), 20);


	positions = zeros(resolution, 9); % Go up
	positions2 = zeros(resolution2, 9); % Climb
	transition = zeros(resolution, 9); % Stab down

	% obj.shank_rotation = -0.17295;

	
	obj.set_head_mass(1e-6);
	obj.configure(x0);

	obj.refresh();

	obj.K1 = 1;
	obj.K2 = 10;
	obj.K3 = 1;
	obj.K4 = 1;
	obj.K5 = 0;

	problem.options = optimoptions('fmincon','StepTolerance',1e-100, 'FunctionTolerance', 1e-50);
	% problem.options = optimoptions('fmincon','FunctionTolerance',1e-100);
	% problem.options = optimoptions('fmincon','UseParallel', 0);

	% problem.options = optimoptions('fmincon','Display','iter');

   % First solve for mode C -> mode B up
   % Second, use First waypoint from first sol as end waypoint to START -> 
   % Third, continue form first solution as usual

	problem.options.MaxFunctionEvaluations = 30000;
	problem.objective = @obj.solve;
	problem.lb = obj.lb;
	problem.ub = obj.ub;
	problem.solver = 'fmincon';

	index = 1
	theta = obj.x0;

	height_limit = 0.35;

	height_trajectory = linspace(0.35, 0.7, resolution);
	obj.obstacle_height = obstacle_height+0.004;
	shank_height_trajectory = linspace(0, obj.obstacle_height, resolution);
	obj.stab_height = obstacle_height;

	problem.ub(2) = 0;
	problem.lb(2) = 0;
	problem.lb(1) = -0.3;
	problem.ub(1) = 0;

	problem.ub(4) = 2;
	problem.lb(4) = 2;

	for i = 1:1:resolution
	  if i==2
	  	theta_begin = theta;
	  end
	  obj.shank_height = shank_height_trajectory(i);
	  theta = obj.run(problem, theta, height_limit);
	  % theta(4) = obj.lhm_position;
	  positions(i, 1) = i;
	  positions(i, 2:9) = obj.convert_to_robot_output(theta);
	  fprintf('lhm command = %g \n',positions(i, 5))
	  fprintf('Table 1 is %g percent complete\n', (i/resolution)*100)
	  obj.refresh;
	end


	theta_temp = theta;

	theta = x0;
	shank_height_temp = obj.shank_height;
	obj.shank_height = 0;
	obj.stab_height = obstacle_height;
    x0 = [-0.7854, 0.3361, 0.0909, 1, 0.3839, 0.3839, 0, 0]
    % x0 = [-0.7854, 0.3361, 0.0909, 0.226007, 0.3839, 0.3839, 0, 0]
	% xT = [-0.135, 0, -0.01, 0.186426, 0, 0, 0, 0]
	x1 = linspace(x0(1), theta_begin(1), 20);
	x2 = linspace(x0(2), theta_begin(2), 20);
	x3 = linspace(x0(3), theta_begin(3), 20);
	% x4 = linspace(x0(4), abs(obj.lhm_position), 20);
	x4 = linspace(1, 1, 20);
	% x5 = linspace(x0(5), theta_begin(5), 20);
	% x6 = linspace(x0(6), theta_begin(6), 20);
	x7 = linspace(x0(7), theta_begin(7), 20);
	x8 = linspace(x0(8), theta_begin(8), 20);
	obj.stab_height = 0.08;

	for i = 1:1:resolution
		problem.lb = [x1(i), x2(i), x3(i), x4(i), -pi, -pi, x7(i), x8(i)];
		problem.ub = [x1(i), x2(i), x3(i), x4(i), pi, pi, x7(i), x8(i)];
		theta = obj.run(problem, theta, 0.35);
		disp(theta)
		% obj.configure([x1(i), x2(i), x3(i), x4(i), x5(i), x6(i), x7(i), x8(i)]);
		% theta(4) = obj.lhm_position;
		obj.refresh();
		transition(i, 1) = i;
		transition(i, 2:9) = obj.convert_to_robot_output(theta);
		fprintf('lhm command = %g\n',transition(i, 5))
		fprintf('Transition Table is %g percent complete\n', (i/resolution)*100)
	end

	theta = theta_temp;

	obj.shank_height = shank_height_temp;

	problem.ub = [0 1 0.8 0 0.383972435 0.383972435 0 0];
	problem.lb = [-pi/4 0 -1 0 -pi -pi 0 0];
	com_height_trajectory = linspace(obstacle_height+0.375, obstacle_height+0.5, resolution2/2);
	com_height_trajectory = [com_height_trajectory, linspace(obstacle_height+0.5, obstacle_height+0.375, resolution2/2)];
	stab_trajectory = linspace(-0.3505, 0, resolution);

	% index = 1;
	lhm_trajectory = linspace(theta(4), 0, resolution2);
	obj.K1 = 1;
	obj.K2 = 10;
	obj.K3 = 100;
	obj.K4 = 1;
	% problem.ub(4) = 0;
	% problem.lb(4) = 0;
	obj.lb(4) = theta(4);
	obj.ub(4) = theta(4);

	% problem.ub(1) = 0;
	% problem.lb(1) = 0;

	% hip_weight_traj = linspace(0.5, 1, resolution2);

	for i = 1:1:resolution2
	  if i > resolution2/2
	  	obj.K4 = 10;

	  end
	  % obj.K4 = hip_weight_traj(i);
	  % obj.K2 = stability_weight_traj(i);

	  obj.lb(4) = lhm_trajectory(i);
	  obj.ub(4) = lhm_trajectory(i);
	  % obj.lb
	  theta = obj.run(problem, theta, com_height_trajectory(i));
	  positions2(i, 1) = i;
  	  % theta(4) = obj.lhm_position;
	  positions2(i, 2:9) = obj.convert_to_robot_output(theta);
	  fprintf('lhm command = %g \n',positions2(i, 5))
	  fprintf('Table 2 is %g percent complete\n', (i/resolution2)*100)
	  obj.refresh;
	end

	stage_1 = transition;

	stage_2 = positions;

	stage_3 = positions2;

% 	waypoints.stage_3 = [0 0.5 0.1 0.25 -0.15 -0.7924 -0.7924 0 0];
	


	fprintf('Table is complete')
end

