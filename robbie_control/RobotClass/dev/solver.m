function theta = solver(x0)
close all; clear all;
obj = LookupTableGenerator();
problem.options = optimoptions('fmincon','Display','iter');
problem.options.MaxFunctionEvaluations = 2000;
problem.objective = @obj.solve;
problem.x0 = obj.x0;
problem.lb = obj.lb;
problem.ub = obj.ub;
problem.solver = 'fmincon';

theta = obj.run(problem, [70 80 50 -70 -70], 0.3)
% theta = obj.run(problem, theta, 0.35)
theta = obj.run(problem, theta, 0.4)
% theta = obj.run(problem, theta, 0.45)
% theta = obj.run(problem, theta, 0.5)
% theta = obj.run(problem, theta, 0.55)
theta = obj.run(problem, theta, 0.6)

end