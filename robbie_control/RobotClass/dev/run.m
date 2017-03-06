close all; clear all; clc;
setup_path;
obj = LookupTableGenerator();
obj.configure([0 0 0.8 -0.2 0 0 0 0])
obj.refresh;
obj.K1 = 1000;
obj.K2 = 1000;
obj.K3 = 1;
% problem.options = optimoptions('fmincon','Display','iter');
problem.options = optimoptions('fmincon');

% problem.options.MaxFunctionEvaluations = 2000;
problem.objective = @obj.solve;
problem.x0 = obj.x0;
problem.lb = obj.lb;
problem.ub = obj.ub;
problem.solver = 'fmincon';
%%

% theta = obj.run(problem, [0 0 0 0 0 0 0 0], 0.3)
% theta = obj.run(problem, theta, 0.35)
% theta = obj.run(problem, theta, 0.4)
% theta = obj.run(problem, theta, 0.45)
% theta = obj.run(problem, theta, 0.5)
% theta = obj.run(problem, theta, 0.55)
% theta = obj.run(problem, theta, 0.8)
% % 
obj.configure([0 0 0.8 -0.2 0 0 0 0]);
obj.refresh;
% z = linspace(0.3, 0.6, 100);

% % pause(2);
% theta = obj.run(problem, theta, 0.35)
% obj.refresh;
% act = convert_to_robot_output(theta)
% theta = obj.run(problem, theta, 0.4);
% obj.refresh;
% act = convert_to_robot_output(theta)
% theta = obj.run(problem, theta, 0.45);
% obj.refresh;
% act = convert_to_robot_output(theta)
% theta = obj.run(problem, theta, 0.5);
% obj.refresh;
% act = convert_to_robot_output(theta)
% theta = obj.run(problem, theta, 0.55);
% obj.refresh;

% theta = obj.run(problem, obj.x0, 0.3);
% obj.refresh;


% act = convert_to_robot_output(theta)
theta = obj.run(problem, obj.x0, 0.5);
obj.refresh;

act = convert_to_robot_output(theta)
theta = obj.run(problem, theta, 0.55);
obj.refresh;

act = convert_to_robot_output(theta)
theta = obj.run(problem, theta, 0.6);
obj.refresh;

act = convert_to_robot_output(theta)
theta = obj.run(problem, theta, 0.65);
obj.refresh;

act = convert_to_robot_output(theta)
theta = obj.run(problem, theta, 0.7);
obj.refresh;

% act = convert_to_robot_output(theta)