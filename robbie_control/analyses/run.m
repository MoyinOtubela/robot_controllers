close all;clear('all');



bag_name = 'aerobot_tf_100w_1.bag'
% bag_name = 'aerobot_tf_1.bag'
obj = aerobot_analysis(bag_name, 1000);

mats = obj.load_tree;

[pose, contacts, locations] = obj.get_coordinates(mats);

stability = obj.analyse_stability(pose.com, contacts, locations);

obj.animate(pose, stability);

[joints, time] = obj.plot_joint_positions(bag_name);

