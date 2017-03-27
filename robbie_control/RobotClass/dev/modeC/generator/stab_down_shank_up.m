function [stage_1, stage_2, stage_3, stage_4] = stab_down_shank_up(x0, resolution, resolution2, obstacle_height)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    clc; close all;
	addpath('../../../');
	addpath('../../');

	obj = LookupTableGenerator();

	positions = zeros(resolution2, 9); % lhm up 
	positions2 = zeros(resolution2, 9); % lhm up 
	positions3 = zeros(resolution2, 9); % lhm up 
	transition = zeros(resolution, 9); % Stab down, contact other side

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

   % First solve for mode C -> to other side
   % Second, use First waypoint from first sol as end waypoint to START -> 
   % Third, continue from first solution as usual to raise lhm up (make trajectory)

	problem.options.MaxFunctionEvaluations = 30000;
	problem.objective = @obj.solve;
	problem.lb = obj.lb;
	problem.ub = obj.ub;
	problem.solver = 'fmincon';

	index = 1
	theta = obj.x0;

	height_limit = 0.35;

	obj.obstacle_height = obstacle_height;
	% shank_height_trajectory = linspace(0, obj.obstacle_height, resolution);
	stab_height_trajectory = linspace(0.08, 0, resolution);
	obj.stab_height = 0.0;


	problem.ub(2) = 0;
	problem.lb(2) = 0;
	problem.lb(1) = -0.3;
	problem.ub(1) = 0;

	problem.ub(4) = 1;
	problem.lb(4) = 1;

	obj.stab_height = 0;
	theta = obj.run(problem, theta, height_limit);
	fprintf('lhm command = %g \n',theta(4))
	obj.refresh;

	theta_begin = theta;

	theta_temp = theta;

	theta = x0;
	% shank_height_temp = obj.shank_height;
	% obj.shank_height = 0;
 %    x0 = [-0.7854, 0.3361, 0.0909, 0.226007, 0.3839, 0.3839, 0, 0]
	% % xT = [-0.135, 0, -0.01, 0.186426, 0, 0, 0, 0]
	x1 = linspace(x0(1), theta_begin(1), 20);
	x2 = linspace(x0(2), theta_begin(2), 20);
	x3 = linspace(x0(3), theta_begin(3), 20);
	x4 = linspace(2, 2, 20);
	% x5 = linspace(x0(5), theta_begin(5), 20);
	% x6 = linspace(x0(6), theta_begin(6), 20);
	x7 = linspace(x0(7), theta_begin(7), 20);
	x8 = linspace(x0(8), theta_begin(8), 20);

            % obj.ub = [0 1 0.8 0 0.383972435 0.383972435 0 0];
            % obj.lb = [-pi/4 0 -1 0 -pi -pi 0 0];
	obj.K1 = 1;
	obj.K2 = 100;
	obj.K3 = 1;
	obj.K4 = 1;
	obj.K5 = 10;

	for i = 1:1:resolution
		problem.lb = [x1(i), x2(i), x3(i), x4(i), -pi/2, -pi/2, 0, 0];
		problem.ub = [x1(i), x2(i), x3(i), x4(i), pi/2, pi/2, 0, 0];
		obj.stab_height = stab_height_trajectory(i);
		theta = obj.run(problem, theta, 0.35);
		% disp(theta)/
		theta(4) = obj.lhm_position;
		obj.refresh();
		transition(i, 1) = i;
		transition(i, 2:9) = obj.convert_to_robot_output(theta);
		fprintf('Transition Table is %g percent complete\n', (i/resolution)*100)
		fprintf('lhm command = %g \n',transition(i, 5))
	end

	% theta = theta_temp;

	% obj.shank_height = shank_height_temp;

	% problem.ub = [0 1 0.8 0 0.383972435 0.383972435 0 0];
	% problem.lb = [-pi/4 0 -1 0 -pi -pi 0 0];
	% com_height_trajectory = linspace(obstacle_height+0.375, obstacle_height+0.5, resolution2/2);
	% com_height_trajectory = [com_height_trajectory, linspace(obstacle_height+0.5, obstacle_height+0.375, resolution2/2)];
	% stab_trajectory = linspace(-0.3505, 0, resolution);

	% % index = 1;
	% lhm_trajectory = linspace(theta(4), 0, resolution2);
	obj.K1 = 1;
	obj.K2 = 10;
	obj.K3 = 100;
	obj.K4 = 1;
	% % problem.ub(4) = 0;
	% % problem.lb(4) = 0;
	obj.lb(4) = theta(4);
	obj.ub(4) = 0.3;

	% % problem.ub(1) = 0;
	% % problem.lb(1) = 0;

	% % hip_weight_traj = linspace(0.5, 1, resolution2);
	shank_height_trajectory = linspace(0, 0.02, resolution2);

	for i = 1:1:resolution2
	  % if i > resolution2/2
	  % 	obj.K4 = 10;
	  % end
	  % obj.K4 = hip_weight_traj(i);
	  % obj.K2 = stability_weight_traj(i);
	  % obj.lb
	  obj.shank_height = shank_height_trajectory(i);
	  theta = obj.run(problem, theta, 0.35);
	  % theta = obj.run(problem, theta, com_height_trajectory(i));
	  positions(i, 1) = i;
  	  % theta(4) = obj.lhm_position;
	  positions(i, 2:9) = obj.convert_to_robot_output(theta);
	  fprintf('Table 2 is %g percent complete\n', (i/resolution2)*100)
	  obj.refresh;
	end

	shank_height_trajectory = linspace(0.02, 0, resolution2);

	for i = 1:1:resolution2
	  if i > resolution2/2
	  	obj.K4 = 10;

	  end
	  % obj.K4 = hip_weight_traj(i);
	  % obj.K2 = stability_weight_traj(i);
	  % obj.lb
	  obj.shank_height = shank_height_trajectory(i);
	  theta = obj.run(problem, theta, 0.35);
	  % theta = obj.run(problem, theta, com_height_trajectory(i));
	  positions2(i, 1) = i;
  	  % theta(4) = obj.lhm_position;
	  positions2(i, 2:9) = obj.convert_to_robot_output(theta);
	  fprintf('Table 2 is %g percent complete\n', (i/resolution2)*100)
	  obj.refresh;
	end

	problem.ub = [0 1 0.8 0 0.383972435 0.383972435 0 0];
	problem.lb = [-pi/4 0 -1 0 -pi -pi 0 0];

	% problem.ub(1) = -0.35;
	% problem.lb(1) = -0.35;

    problem.ub(4) = 2;
    problem.lb(4) = 2;
    % problem.ub(2) = 0;
    % problem.lb(2) = 0;

	% shank_trajectory = linspace(-0.35, 0, resolution2);
	shank_trajectory = linspace(theta(1), 0, resolution2);
	% lhm_trajectory = linspace(-theta(4), 0, resolution2);
	% height_trajectory = linspace(0.3, 0.35, resolution2);
	% knee_trajectory = linspace(theta(4), 0, resolution2);

	% hip_angle_weight_trajectory = linspace(0.1, 1, resolution2);
	% hip_angle_weight_trajectory = linspace(0.5, 0.5, resolution2);
	% stability_weight_trajectory = linspace(100, 1, resolution2);
	% stability_weight_trajectory_2 = linspace(1, 0, resolution2);
	count = 1;

	stop_condition = resolution2*0.5;

	iter = 0;

	obj.shank_height = 0;

	obj.K5 = 0;

	obj.K4 = 10;
	obj.K2 = 100;
	% obj.K4 = 10;
	problem.objective = @obj.solve_stab_shank;


	for i = 1:1:resolution2

	  if i >= stop_condition
	    problem.lb(4) = 0;
	    problem.ub(4) = 0;
	    % obj.K4 = 10;
	  end
	  % count = count + 1;
	  % problem.ub(4) = lhm_trajectory(i);
	  % problem.lb(4) = lhm_trajectory(i);
	  % obj.K2 = stability_weight_trajectory(i);
	  % obj.K5 = stability_weight_trajectory_2(i);
	  % obj.K4 = hip_angle_weight_trajectory(i);
	  % obj.K3 = height_weight_trajectory(i);
	  problem.lb(1) = shank_trajectory(i);
	  problem.ub(1) = shank_trajectory(i);
	  theta = obj.run(problem, theta, 0.3);
	  % theta = obj.run(problem, theta, height_trajectory(i));
	  % theta(4) = -abs(theta(4));
	  fprintf('theta(4) = %g\n', theta(4))
	  positions3(i, 1) = i;
	  positions3(i, 2:9) = obj.convert_to_robot_output(theta);
	  fprintf('Table 3 is %g percent complete\n', (i/resolution2)*100)
	  fprintf('lhm command = %g\n',positions3(i, 5))
	  obj.refresh;
	  % fprintf('hip angle %f\n', obj.hip_monitor*180/pi);
	end


	stage_1 = transition;

	stage_2 = positions;

	stage_3 = positions2;

	stage_4 = positions3;



	fprintf('Table is complete')
end


	% positions = zeros(resolution, 9);

	% obj = LookupTableGenerator();
	% obj.set_head_mass(1e-6);

	% obj.K1 = 1;
	% obj.K2 = 10;
	% obj.K3 = 1e2;
	% obj.K4 = 1000;
	% obj.K5 = 0;

	% problem.options = optimoptions('fmincon','StepTolerance',1e-100, 'FunctionTolerance',1e-100);
	% problem.options.MaxFunctionEvaluations = 30000;
	% problem.objective = @obj.solve;
	% problem.lb = obj.lb;
	% problem.ub = obj.ub;
	% problem.solver = 'fmincon';

	% height_limit = 0.35;

	% theta = obj.x0;
	% height_trajectory = linspace(0.35, 0.4, resolution);

	% obj.hip_limit = pi/2.2;

 %    problem.ub(4) = 1;
 %    problem.lb(4) = 1;

	% positions = zeros(1, 9);
	% positions2 = zeros(resolution, 9);
	% positions3 = zeros(resolution2, 9);
	% positions4 = zeros(resolution2, 9);


	% obj.stab_height = obstacle_height;
	% theta = obj.run(problem, theta, height_limit);
	% obj.refresh;
	% positions = zeros(1, 9);
	% positions(1,1) = 0;
	% positions(1,2:9) = obj.convert_to_robot_output(theta);

	% % obj.com_height_limit = height_limit;

	% % stab_trajectory = linspace(0.2, 0, resolution);
	% shank_height_trajectory = linspace(0, 0.03, resolution);


	% % -0.3505 = 0.8 stab_angle
	% obj.K1 = 1;
	% obj.K2 = 0.7e0;
	% obj.K3 = 10;
	% obj.K4 = 100;

	% % Extend lhm fully
 %    problem.ub(4) = 0.4;
 %    problem.lb(4) = 0.4;
 %    % problem.ub(3) = 0;
 %    % problem.lb(3) = -0.5;
	% % problem.ub(1) = -0.5;
	% % problem.lb(1) = -0.5;
	% % problem.ub(1) = 0;
	% % problem.lb(1) = -0.35;
	% problem.ub(1) = -0.35;
	% problem.lb(1) = -0.35;

	% problem.ub(2) = 0;
	% problem.lb(2) = 0;

	% % obj.hip_limit = pi/

	% for i = 1:1:resolution
	%   obj.shank_height = shank_height_trajectory(i);
	%   % obj.shank_distance = shank_distance_trajectory(i);
	%   % theta = obj.run(problem, theta, height_limit);
	%   theta = obj.run(problem, theta, height_trajectory(i));
	%   % theta(4) = -abs(theta(4));
	%   fprintf('theta(4) = %g\n', theta(4))
	%   positions2(i, 1) = i;
	%   positions2(i, 2:9) = obj.convert_to_robot_output(theta);
	%   fprintf('Table 1 is %g percent complete\n', (i/resolution)*100)
	%   obj.refresh;
	%   % fprintf('hip angle %f\n', obj.hip_monitor*180/pi);
	% end

	% shank_height_trajectory = linspace(0.03, 0, resolution2);

	% problem.ub(1) = -0.35;
	% problem.lb(1) = -0.35;

	% for i = 1:1:resolution2
	%   obj.shank_height = shank_height_trajectory(i);
	%   theta = obj.run(problem, theta, height_limit);
	%   fprintf('theta(4) = %g\n', theta(4))
	%   theta(4) = -abs(theta(4));
	%   positions3(i, 1) = i;
	%   positions3(i, 2:9) = obj.convert_to_robot_output(theta);
	%   fprintf('Table 2 is %g percent complete\n', (i/resolution2)*100)
	%   obj.refresh;
	%   % fprintf('hip angle %f\n', obj.hip_monitor*180/pi);
	% end

	% obj.K1 = 1;
	% obj.K2 = 1e3;
	% obj.K3 = 1e2;
	% obj.K4 = 0.5;


	% height_limit = 0.375

	% problem.ub = [0 1 0.8 0 0.383972435 0.383972435 0 0];
	% problem.lb = [-pi/4 0 -1 0 -pi -pi 0 0];

	% % problem.ub(1) = -0.35;
	% % problem.lb(1) = -0.35;

 %    problem.ub(4) = 0.25;
 %    problem.lb(4) = 0.25;
 %    problem.ub(2) = 0;
 %    problem.lb(2) = 0;

	% shank_trajectory = linspace(-0.35, 0, resolution2);
	% lhm_trajectory = linspace(-theta(4), 0, resolution2);
	% height_trajectory = linspace(0.3, 0.35, resolution2);
	% % knee_trajectory = linspace(theta(4), 0, resolution2);

	% hip_angle_weight_trajectory = linspace(1, 5, resolution2);
	% % hip_angle_weight_trajectory = linspace(0.5, 0.5, resolution2);
	% height_weight_trajectory = linspace(1e-1, 10, resolution2);
	% stability_weight_trajectory = linspace(1e3, 1e1, resolution2);
	% count = 1;

	% stop_condition = resolution2*0.3;

	% iter = 0;

	% for i = 1:1:resolution2

	%   if i >= stop_condition
	%     problem.lb(4) = 0;
	%     problem.ub(4) = 0;

	%   end
	%   % count = count + 1;
	%   % problem.ub(4) = lhm_trajectory(i);
	%   % problem.lb(4) = lhm_trajectory(i);
	%   obj.K2 = stability_weight_trajectory(i);
	%   obj.K4 = hip_angle_weight_trajectory(i);
	%   % obj.K3 = height_weight_trajectory(i);
	%   problem.lb(1) = shank_trajectory(i);
	%   problem.ub(1) = shank_trajectory(i);
	%   theta = obj.run(problem, theta, height_trajectory(i));
	%   theta(4) = -abs(theta(4));
	%   fprintf('theta(4) = %g\n', theta(4))
	%   positions4(i, 1) = i;
	%   positions4(i, 2:9) = obj.convert_to_robot_output(theta);
	%   fprintf('Table 3 is %g percent complete\n', (i/resolution2)*100)
	%   obj.refresh;
	%   % fprintf('hip angle %f\n', obj.hip_monitor*180/pi);
	% end

	% stage_1 = positions;

	% stage_2 = positions2;

	% stage_3 = positions3;

	% stage_4 = positions4;
