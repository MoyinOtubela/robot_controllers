close all;clear all;

% bagfiles
% bag_name = 'autonomous_step_climbing.bag'
% bag_name = 'autonomous_crevice_crossing.bag'

bag_name = 'manual_step_climbing.bag'
% bag_name = 'crevice_crossing.bag'

obj = aerobot_analysis_2(bag_name, 1000);
% obj.ground = [0, 0.2];
obj.ground = [0, 0.08];
mats = obj.load_tree;

[pose, contacts, locations] = obj.get_coordinates(mats);

obj.animate(pose);

[joints, time_j] = obj.plot_joint_positions('/robbie/whole_body_controller/state');

[joints_errors, time_e] = obj.plot_joint_error_positions('/robbie/whole_body_controller/state');

[joints_velocities, time_v] = obj.plot_joint_velocities_positions('/robbie/whole_body_controller/state');
[joints_velocities_error, time_ve] = obj.plot_joint_error_velocities_positions('/robbie/whole_body_controller/state');
ssm_topic.ssm = '/robbie/ssm'; 
ssm_topic.ssm_delta = '/robbie/ssm_delta'; 

stability = obj.analyse_stability(pose.com, contacts, locations, time_j);
[stability_2, time] = obj.plot_stability(ssm_topic);

