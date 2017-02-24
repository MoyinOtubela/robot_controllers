% 
close all; clear; clc;

%% SETUP PHYSICAL ROBOT PARAMS
% shank params
shank_len = 1.2;
shank_width = 1;
drive_wheel_rad = 0.25;

% stabiliser params
stab_knee_offset = 0.2;
stab_len = 0.92;
stab_wheel_rad = 0.2;

% thigh params
thigh_len = 0.65;

% torso params
torso_len = 1.75;
torso_width = 0.8;
bicep_len = 0.45;
arm_ext = 1;

% LHM params
lhm_width = 0.9;
lhm_elongation = 0.5;
lhm_wheel_rad = 0.25;

% joint angles
shank_angle = 30;
knee_angle = 45;
hip_angle = 0;
r_arm_angle = -90; % negative means arm down, positive meanse arm up
l_arm_angle = -90; % negative means arm down, positive meanse arm up

% note: all angles are in degrees

   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Joint_A =  -43;
Joint_B =   97;
Joint_C = -53;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 

  

  

%% stabiliser parameters

k = 90 + Joint_A;
h = (shank_len - stab_knee_offset)*sind(k);

% if (stab_len < h+(drive_wheel_rad-stab_wheel_rad))
%  stab_len = h+(drive_wheel_rad-stab_wheel_rad);
% end

stab_angle = k +  asind((h+(drive_wheel_rad-stab_wheel_rad))/(stab_len));

%% SETUP COORDINATE SYSTEM %%





% Origin
origin = [0 0 0 1]';

% LOWER BODY
% origin-r_wheel 
a_1 = 0;
alpha_1 = 0;
d_1 = shank_width/2;
theta_1 = 0;
[A1, A1_sym] = DH(a_1, alpha_1, d_1, theta_1);

% origin-l_wheel 
a_2 = 0;
alpha_2 = 0;
d_2 = -(shank_width/2);
theta_2 = 0;
[A2, A2_sym] = DH(a_2, alpha_2, d_2, theta_2);

% origin-stabiliser 
a_3 = shank_len-stab_knee_offset;
alpha_3 = 0;
d_3 = 0;
theta_3 = Joint_A;
[A3, A3_sym] = DH(a_3, alpha_3, d_3, theta_3);

% stabiliser_joint-omniwheel
a_4 = stab_len;
alpha_4 = 0;
d_4 = 0;
theta_4 = -stab_angle;
[A4, A4_sym] = DH(a_4, alpha_4, d_4, theta_4);

% origin-knee  
a_5 = shank_len;
alpha_5 = 0;
d_5 = 0;
theta_5 = Joint_A;
[A5, A5_sym] = DH(a_5, alpha_5, d_5, theta_5);

% knee-hip
a_6 = thigh_len;
alpha_6 = 0;
d_6 = 0;
theta_6 = Joint_B;
[A6, A6_sym] = DH(a_6, alpha_6, d_6, theta_6);

% UPPER BODY
% hip-neck
a_7 = torso_len;
alpha_7 = 0;
d_7 = 0;
theta_7 = Joint_C;
[A7, A7_sym] = DH(a_7, alpha_7, d_7, theta_7);

% neck-r_shoulder
a_8 = 0;
alpha_8 = 0;
d_8 = torso_width/2;
theta_8 = 0;
[A8, A8_sym] = DH(a_8, alpha_8, d_8, theta_8);

% neck-l_shoulder
a_9 = 0;
alpha_9 = 0;
d_9 = -(torso_width/2);
theta_9 = 0;
[A9, A9_sym] = DH(a_9, alpha_9, d_9, theta_9);

% r_shoulder-r_elbow
a_10 = 0;
alpha_10 = 90;   % by design 
d_10 = 0;
theta_10 = r_arm_angle;
[A10, A10_sym] = DH(a_10, alpha_10, d_10, theta_10);

    % default arm length
a_11 = 0;
alpha_11 = 0;
d_11 = bicep_len + arm_ext;
theta_11 = 0;
[A11, A11_sym] = DH(a_11, alpha_11, d_11, theta_11);

%{
    % arm extension
a_10a = 0;
alpha_10a = 0;
d_10a = arm_ext;
theta_10a = 0;
[A10a, A10a_sym] = DH(a_10a, alpha_10a, d_10a, theta_10a);
%}

% l_shoulder-l_elbow
a_12 = 0;
alpha_12 = 90;  % by design 
d_12 = 0;
theta_12 = l_arm_angle;
[A12, A12_sym] = DH(a_12, alpha_12, d_12, theta_12);

    % default arm length
a_13 = 0;
alpha_13 = 0;
d_13 = bicep_len + arm_ext;
theta_13 = 0;
[A13, A13_sym] = DH(a_13, alpha_13, d_13, theta_13);

%{
    % arm extension
a_12a = 0;
alpha_12a = 0;
d_12a = arm_ext;
theta_12a = 0;
[A12a, A12a_sym] = DH(a_12a, alpha_12a, d_12a, theta_12a);
%}

% LOWER HIP MECHANISM
% hip-lhm
% first reorientate the coordinate frame
a_14 = 0;
alpha_14 = 90;  % by design 
d_14 = 0;
theta_14 = -90-(-Joint_C)-(Joint_B);
[A14, A14_sym] = DH(a_14, alpha_14, d_14, theta_14);

% translate coordinate frame from centre of hip
a_15 = -0.15;   % hip offset
alpha_15 = 0;
d_15 = lhm_elongation;    
theta_15 = 0;
[A15, A15_sym] = DH(a_15, alpha_15, d_15, theta_15);

% lhm_origin
% reorientate the coordinate frame
a_16 = 0;
alpha_16 = -90; % by design
d_16 = 0;
theta_16 = 0;
[A16, A16_sym] = DH(a_16, alpha_16, d_16, theta_16);

a_17 = 0;
alpha_17 = 0;
d_17 = 0;
theta_17 = 90;  % by design
[A17, A17_sym] = DH(a_17, alpha_17, d_17, theta_17);

% lhm_origin-lhm_rwheel
a_18 = 0;
alpha_18 = 0;
d_18 = (lhm_width/2);
theta_18 = 0;
[A18, A18_sym] = DH(a_18, alpha_18, d_18, theta_18);

% lhm_origin-lhm_lwheel
a_19 = 0;
alpha_19 = 0;
d_19 = -(lhm_width/2);
theta_19 = 0;
[A19, A19_sym] = DH(a_19, alpha_19, d_19, theta_19);




%% GET CO-0RDINATES 
% lower body
wheel_rhs = A1*origin
wheel_lhs = A2*origin
stabiliser = A3*origin
stabiliser_wheel = A3*A4*origin

% upper body  
knee = A5*origin
hip = A5*A6*origin
neck = A5*A6*A7*origin
shoulder_rhs = A5*A6*A7*A8*origin
shoulder_lhs = A5*A6*A7*A9*origin
elbow_rhs = A5*A6*A7*A8*A10*A11*origin
elbow_lhs = A5*A6*A7*A9*A12*A13*origin

% lower hip mechanism
lhm = A5*A6*A14*A15*origin
lhm_rwheel = A5*A6*A14*A15*A16*A17*A18*origin
lhm_lwheel = A5*A6*A14*A15*A16*A17*A19*origin

%% VISUALISE ROBOT

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot robot
coord = zeros(15,3);
coord(1,1:3) = origin(1:3);
coord(2,1:3) = wheel_rhs(1:3);
coord(3,1:3) = wheel_lhs(1:3);
coord(4,1:3) = stabiliser(1:3);
coord(5,1:3) = knee(1:3);
coord(6,1:3) = hip(1:3);
coord(7,1:3) = neck(1:3);
coord(8,1:3) = shoulder_rhs(1:3);
coord(9,1:3) = shoulder_lhs(1:3);
coord(10,1:3) = elbow_rhs(1:3);
coord(11,1:3) = elbow_lhs(1:3);
coord(12,1:3) = lhm(1:3);
coord(13,1:3) = lhm_rwheel(1:3);
coord(14,1:3) = lhm_lwheel(1:3);
coord(15,1:3) = stabiliser_wheel(1:3);

% setup figure
title('Robbie Kinematics','FontSize',12);
figure(1);
axis([-2 2 -2 2 -0.25 3]); hold on;
xlabel('X-axis');ylabel('Y-axis');zlabel('Z-axis');
grid on
% plot (coord(1:5,2),coord(1:5,1))

% plot coordinates
plot3(coord(1:15,2),coord(1:15,3),coord(1:15,1),'o');
hold on;

% draw lines

% wheelbase
plot3(coord(2:3,2),coord(2:3,3),coord(2:3,1),'r','LineWidth',3);
hold on;

% draw drive_wheels
plotCircle3D( [ wheel_rhs(2), wheel_rhs(3), wheel_rhs(1) ] ,[0, 1, 0], drive_wheel_rad, 'g')
plotCircle3D( [ wheel_lhs(2), wheel_lhs(3), wheel_lhs(1) ] ,[0, 1, 0], drive_wheel_rad, 'g')
hold on; 

% shank
plot3( [coord(1,2);coord(5,2)],[coord(1,3); coord(5,3)],[coord(1,1); coord(5,1)],'r','LineWidth',3);
hold on;

% stabiliser
plot3( [coord(4,2);coord(15,2)],[coord(4,3); coord(15,3)],[coord(4,1); coord(15,1)],'r','LineWidth',3);
hold on;
plotCircle3D( [ stabiliser_wheel(2), stabiliser_wheel(3), stabiliser_wheel(1) ] ,[0, 1, 0], stab_wheel_rad, 'r')

% thigh
plot3(coord(5:6,2),coord(5:6,3),coord(5:6,1),'r','LineWidth',3);
hold on;

% torso
plot3(coord(6:7,2),coord(6:7,3),coord(6:7,1),'r','LineWidth',3);
hold on;

% shoulderwidth
plot3(coord(8:9,2),coord(8:9,3),coord(8:9,1),'r','LineWidth',3);
hold on;

% right arm
plot3( [coord(8,2);coord(10,2)],[coord(8,3); coord(10,3)],[coord(8,1); coord(10,1)],'r','LineWidth',3);
hold on;

% left arm
plot3( [coord(9,2);coord(11,2)],[coord(9,3); coord(11,3)],[coord(9,1); coord(11,1)],'r','LineWidth',3);
hold on;

% hip-lhm
plot3( [coord(6,2);coord(12:14,2)],[coord(6,3); coord(12:14,3)],[coord(6,1); coord(12:14,1)],'b','LineWidth',3);
hold on;
plotCircle3D( [ lhm_rwheel(2), lhm_rwheel(3), lhm_rwheel(1) ] ,[0, 1, 0], lhm_wheel_rad)
plotCircle3D( [ lhm_lwheel(2), lhm_lwheel(3), lhm_lwheel(1) ] ,[0, 1, 0], lhm_wheel_rad)
hold on;














