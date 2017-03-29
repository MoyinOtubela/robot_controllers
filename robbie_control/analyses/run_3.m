close all;clear all;

% bagfiles
% bag_name = 'autonomous_step_climbing.bag'
% bag_name = 'autonomous_crevice_crossing.bag'
% bag_name = 'manual_crevice_crossing.bag'
bag_name = 'manual_step_climbing.bag'
% bag_name = 'manual_crevice_crossing.bag'
% bag_name = 'height_adjust.bag'

% bag_name = 'manual_step_climbing.bag'
% bag_name = 'test_step_climbing.bag'
% bag_name = 'test_crevice_crossing.bag'
% bag_name = 'modeA.bag'
% bag_name = 'modeB.bag'
% bag_name = 'modeC.bag'
% bag_name = 'modeD.bag'
% bag_name = 'step_climbing.bag'
% bag_name = 'crevice_crossing.bag'

obj = aerobot_analysis_3(bag_name);
% Get positions, orienations and contact sensor data
[mats, contact, time] = obj.load_gazebo_data('/robbie/LocationContact');
disp(time)
% 

[pose, locations] = obj.get_coordinates(mats);

obj.resolution = length(mats.shank_footprint.position)

% % shank_footprint_com = obj.transform_point(mats.shank_footprint, [0.163037 0 0.159341 1]')
% % stab_wheel_com = obj.transform_point(mats.stab_wheel, [0 0 0 1]')
% obj.animate(pose);

[joints, time_j] = obj.plot_joint_positions('/robbie/whole_body_controller/state');

[joints_errors, time_e] = obj.plot_joint_error_positions('/robbie/whole_body_controller/state');
[joints_velocities, time_v] = obj.plot_joint_velocities_positions('/robbie/whole_body_controller/state');
[joints_velocities_error, time_ve] = obj.plot_joint_error_velocities_positions('/robbie/whole_body_controller/state');



obj.plot_contacts(contact, time)

stability = obj.analyse_stability(pose.com, contact, locations, time);


% % ssm_topic.ssm = '/robbie/ssm'; 
% % ssm_topic.ssm_delta = '/robbie/ssm_delta'; 

% [stability_2, time] = obj.plot_stability(ssm_topic);

