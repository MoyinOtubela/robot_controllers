clc
clear 

f = v2_robot(1);
f.animate()
% f.setupDH();

%% Desired ee location
% desired_ee = [x, y, z]';   %required end effector position
desired_ee = [3, -0.3, 0]';   %required end effector position
f.ikine(desired_ee)
% f.theta_5
% f.theta_6
% f.theta_7f