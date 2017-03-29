classdef aerobot_analysis_3
	properties
		rosbag_dir = '/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/rosbags/'
		rosbag;

		world_frame = 'odom';

        shank_mass = 7;
        stab_mass = 1;
        % stab_mass = 0.04625;
        stab_wheel_mass = 0.75;
        thigh_mass = 2;
        torso_mass = 16;
        lhm_mass = 2.25;
        left_shoulder_mass = 0.25;
        right_shoulder_mass = 0.25;
        left_arm_mass = 0.25;
        right_arm_mass = 0.25;
        shank_left_wheel_mass = 0.25;
        shank_right_wheel_mass = 0.25;
        left_lhm_wheel_mass = 0.2;
        right_lhm_wheel_mass = 0.2;
        camera_mass = 1e-6;
        total_mass;

        times = [];

        shank_wheel_radius = 0.100886;
        stab_wheel_radius = 0.0762;
        lhm_wheel_radius = 0.102175;

        s;
        h;
        mass;

        ground = [0, 0.08];

        resolution = 100;

	end

	methods

		function obj = aerobot_analysis_3(bag_name)
			close all;
			addpath('/home/moyin/dev/autonomous_controllers/src/matlab_rosbag');
			% obj.times = zeros(1, resolution);
			obj.rosbag = ros.Bag.load(strcat(obj.rosbag_dir,bag_name));

			disp(obj.rosbag.info());
		end

		function [result, time] = plot_stability(obj, topic)
			[ssm_, t1] = obj.rosbag.readAll(topic.ssm);
			[ssm_delta_, t2] = obj.rosbag.readAll(topic.ssm_delta);
			ssm = zeros(1, length(ssm_));
			ssm_delta = zeros(1, length(ssm_delta_));
			time_1 = [];
			time_2 = [];

			for i = 1:length(ssm_)
				ssm(1, i) = ssm_{i};
				time_1 = [time_1; t1{i}.time.time];
			end

			for i = 1:length(ssm_delta_)
				ssm_delta(1, i) = ssm_delta_{i};
				time_2 = [time_2; t2{i}.time.time];
			end

			figure, plot(time_1, ssm), hold on, 
			plot(time_2, ssm_delta), legend('ssm', 'ssm\_delta'), grid on, title('Stability Analysis (topic)'), xlabel('time (s)'), ylabel(''), ylim([0 0.2])
;
			hold off
			result.ssm = ssm;
			result.ssm_delta = ssm_delta;

			time.t1 = time_1;
			time.t2 = time_2;

		end



		function [joints, time] = plot_joint_positions(obj, topic)
			[pos, t] = obj.rosbag.readAll(topic);
			joints = zeros(8, length(pos));
			time = [];
			for i = 1:length(pos)
				joints(1, i) = pos{i}.actual.positions(1);
				joints(2, i) = pos{i}.actual.positions(2);
				joints(3, i) = pos{i}.actual.positions(3);
				joints(4, i) = pos{i}.actual.positions(4);
				joints(5, i) = pos{i}.actual.positions(5);
				joints(6, i) = pos{i}.actual.positions(6);
				joints(7, i) = pos{i}.actual.positions(7);
				joints(8, i) = pos{i}.actual.positions(8);
				time = [time; t{i}.time.time];
			end

			figure, plot(time, joints), 
			grid on, title('Joint positions vs Time'), xlabel('time (s)'), ylabel('joint position'), legend('stabilser (rad)','thigh (rad)','hip (rad)','lhm (m)','left shoulder (rad)','right shoulder (rad)','left elbow (m)','right elbow (m)');
			hold off
		end

		function [joints, time] = plot_joint_velocities_positions(obj, topic)
			[pos, t] = obj.rosbag.readAll(topic);
			joints = zeros(8, length(pos));
			time = [];
			for i = 1:length(pos)
				joints(1, i) = pos{i}.actual.velocities(1);
				joints(2, i) = pos{i}.actual.velocities(2);
				joints(3, i) = pos{i}.actual.velocities(3);
				joints(4, i) = pos{i}.actual.velocities(4);
				joints(5, i) = pos{i}.actual.velocities(5);
				joints(6, i) = pos{i}.actual.velocities(6);
				joints(7, i) = pos{i}.actual.velocities(7);
				joints(8, i) = pos{i}.actual.velocities(8);
				time = [time; t{i}.time.time];
			end

			figure, plot(time, joints), 
			grid on, title('Joint velocities vs Time'), xlabel('time (s)'), ylabel('joint velocities'), legend('stabilser (rad)','thigh (rad)','hip (rad)','lhm (m)','left shoulder (rad)','right shoulder (rad)','left elbow (m)','right elbow (m)');
			
			hold off
		end

		function [joints, time] = plot_joint_error_velocities_positions(obj, topic)
			[pos, t] = obj.rosbag.readAll(topic);
			joints = zeros(8, length(pos));
			time = [];
			for i = 1:length(pos)
				joints(1, i) = pos{i}.error.velocities(1);
				joints(2, i) = pos{i}.error.velocities(2);
				joints(3, i) = pos{i}.error.velocities(3);
				joints(4, i) = pos{i}.error.velocities(4);
				joints(5, i) = pos{i}.error.velocities(5);
				joints(6, i) = pos{i}.error.velocities(6);
				joints(7, i) = pos{i}.error.velocities(7);
				joints(8, i) = pos{i}.error.velocities(8);
				time = [time; t{i}.time.time];
			end

			figure, plot(time, joints), 
			grid on, title('Joint velocities error vs Time'), xlabel('time (s)'), ylabel('joint velocities error'), legend('stabilser (rad)','thigh (rad)','hip (rad)','lhm (m)','left shoulder (rad)','right shoulder (rad)','left elbow (m)','right elbow (m)');
			
			hold off
		end

		function [joints, time] = plot_joint_error_positions(obj, topic)
			[pos, t] = obj.rosbag.readAll(topic);
			joints = zeros(8, length(pos));
			time = [];
			for i = 1:length(pos)
				joints(1, i) = pos{i}.error.positions(1);
				joints(2, i) = pos{i}.error.positions(2);
				joints(3, i) = pos{i}.error.positions(3);
				joints(4, i) = pos{i}.error.positions(4);
				joints(5, i) = pos{i}.error.positions(5);
				joints(6, i) = pos{i}.error.positions(6);
				joints(7, i) = pos{i}.error.positions(7);
				joints(8, i) = pos{i}.error.positions(8);
				time = [time; t{i}.time.time];
			end

			figure, plot(time, joints), 
			grid on, title('Joint error positions vs Time'), xlabel('time (s)'), ylabel('joint position error'), legend('stabilser (rad)','thigh (rad)','hip (rad)','lhm (m)','left shoulder (rad)','right shoulder (rad)','left elbow (m)','right elbow (m)'), ylim([-0.03, 0.03]);
			
			hold off
		end

		function rot = get_rotation_matrix(obj, q)
			rot = zeros(3,3);
			% x = 1;
			% y = 2; 
			% z = 3;
			% w = 4;
			rot(1,1) = 1 - 2*q(2)^2 - 2*q(3)^2;
			rot(1,2) = 2*q(1)*q(2) - 2*q(3)*q(4);
			rot(1,3) = 2*q(1)*q(3) + 2*q(2)*q(4);
			rot(2,1) = 2*q(1)*q(2) + 2*q(3)*q(4);
			rot(2,2) = 1 - 2*q(1)^2 - 2*q(3)^2;
			rot(2,3) = 2*q(2)*q(3) - 2*q(1)*q(4);
			rot(3,1) = 2*q(1)*q(3) - 2*q(2)*q(4);
			rot(3,2) = 2*q(2)*q(3) + 2*q(1)*q(4);
			rot(3,3) = 1 - 2*q(1)^2 - 2*q(2)^2;
		end

		function pos = transform_point(obj, m, point)

			pos = [];
			% disp(m)
			for i = 1:1:length(m.position)
				mat =  ones(4,4);
				mat(1:3, 1:3) = obj.get_rotation_matrix(m.orientation(1:4, i)); 
				mat(1:3,4) = m.position(1:3, i);
				% disp(m.position(1:3, i))
			    coord =  mat*point;
			    pos = [pos;coord(1:3)'];
			end

		end

		% function matrix = transform(obj, mat, rel)

		% 	T1 = ones(4, 4);
		% 	T1(1:3, 4) = rel;


		% 	for i = 1:1:length(m.position)
		% 		mat =  ones(4,4);
		% 		mat(1:3, 1:3) = obj.get_rotation_matrix(m.orientation(1:4, i)); 
		% 		mat(1:3,4) = m.position(1:3, i);
		% 	    coord =  mat*point;
		% 	    pos = [pos;coord(1:3)'];

		% 	end
		% end

		function [pose, locations] = get_coordinates(obj, mats)

			% mats = obj.load_tree;
			% pose.camera_com = obj.transform_point(mats.camera_link, [0 0 0 1]');
			pose.camera_com = obj.transform_point(mats.torso_link, [0, 0, 0.42, 1]');
			pose.shank_footprint_com = obj.transform_point(mats.shank_footprint, [0.163037 0 0.159341 1]');
			pose.stab_wheel_com = obj.transform_point(mats.stab_wheel, [0 0 0 1]');
			pose.stab_link_com = obj.transform_point(mats.stab_link, [0 0 0.04625 1]');
			pose.thigh_link_com = obj.transform_point(mats.thigh_link, [0 0 0.1815 1]');
			pose.torso_link_com = obj.transform_point(mats.torso_link, [0 0 0.192 1]');
			pose.lhm_link_com = obj.transform_point(mats.lhm_link, [0 0 0.02 1]');
			pose.lhm_wheel_left_link_com = obj.transform_point(mats.lhm_wheel_left_link, [0 0 0 1]');
			pose.lhm_wheel_right_link_com = obj.transform_point(mats.lhm_wheel_right_link, [0 0 0 1]');
			pose.shoulder_left_link_com = obj.transform_point(mats.shoulder_left_link, [0 0.16 -0.15 1]');
			pose.shoulder_right_link_com = obj.transform_point(mats.shoulder_right_link, [0 -0.16 -0.15 1]');
			pose.arm_left_link_com = obj.transform_point(mats.arm_left_link, [0 0 -0.1325 1]');
			pose.arm_right_link_com = obj.transform_point(mats.arm_right_link, [0 0 -0.1325 1]');
			pose.wheel_left_link_com = obj.transform_point(mats.wheel_left_link, [0 0 0 1]');
			pose.wheel_right_link_com = obj.transform_point(mats.wheel_right_link, [0 0 0 1]');

			disp(pose)
			% disp(pose.shank_footprint_com)
            mass = [
                obj.camera_mass;
				obj.shank_mass;
                obj.stab_wheel_mass;
                obj.stab_mass;
                obj.thigh_mass;
                obj.torso_mass;
                obj.lhm_mass;
                obj.left_lhm_wheel_mass;
                obj.right_lhm_wheel_mass;
                obj.left_shoulder_mass;
                obj.right_shoulder_mass;
                obj.left_arm_mass;
                obj.right_arm_mass;
                obj.shank_left_wheel_mass;
                obj.shank_right_wheel_mass;
                ];

            total_mass = sum(mass);
             % objective: find overall center of mass for each time frame -> pos.com

			fields = fieldnames(pose);

			disp(fields)

			pose.com = zeros(3, length(mats.shank_footprint.position));

			% fields;

			for j = 1:length(mats.shank_footprint.position)
				for i = 1:15
					pose_ = pose.(fields{i})(j, :);
					% disp(pose_)
					pose.com(1, j) = pose_(1)*mass(i) + pose.com(1, j);
					pose.com(2, j) = pose_(2)*mass(i) + pose.com(2, j);
					pose.com(3, j) = pose_(3)*mass(i) + pose.com(3, j);
				end
				pose.com(1, j) = pose.com(1, j)/total_mass;
				pose.com(2, j) = pose.com(2, j)/total_mass;
				pose.com(3, j) = pose.com(3, j)/total_mass;
			end

			locations.shank = [mats.shank_footprint.position];
			locations.stab = [mats.stab_wheel.position];
			locations.lhm = [mats.lhm_link.position];

			% stability = obj.analyse_stability(pose.com, supports, locations);

			% disp(stability.ssm)
			% % times = linspace(obj.tree.time_begin + 1, obj.tree.time_end - 1, obj.resolution);
			% fig = figure(2)


		end

		function stability = analyse_stability(obj, com, supports, locations, time)
			% stability = 0;

			[stability.ssm, times_ssm] = obj.ssm(com, supports, locations, time);
			[stability.ssm_delta, times_ssm_delta] = obj.ssm_delta(com, supports, locations, time);

			% times_ssm = linspace(time(1), time(end), length(stability.ssm));
			% times_ssm_delta = linspace(time(1), time(end), length(stability.ssm_delta));

			figure, plot(times_ssm, stability.ssm), title('Stability analysis'), xlabel('time (s)'), ylabel('m'), hold on, grid on,
			plot(times_ssm_delta, stability.ssm_delta), legend('ssm','ssm\_delta')
			hold off

		end

		function [result, time] = ssm_delta(obj, com, supports, locations, time_)
			result = [];
			time = [];
			for i = 1:obj.resolution
				if( supports.lhm(i) && supports.stab(i) )
					x0 =  (locations.stab(1, i) +  locations.lhm(1, i) )/2;
					y0 =  (locations.stab(2, i) +  locations.lhm(2, i) )/2;

					result = [result; sqrt(  ( x0 - com(1, i) )^2  + ( y0 - com(2, i) )^2 )];
					time = [time, time_(i)];

				elseif( supports.stab(i) && supports.shank(i) )
					% shank -> upper
					% stab  -> lower
					x0 =  (locations.shank(1, i) +  locations.stab(1, i) )/2;
					y0 =  (locations.shank(2, i) +  locations.stab(2, i) )/2;
					time = [time, time_(i)];

					result = [result; sqrt(  ( x0 - com(1, i) )^2  + ( y0 - com(2, i) )^2 )];
				elseif( supports.lhm(i) && supports.shank(i) )
					x0 =  (locations.shank(1, i) +  locations.lhm(1, i) )/2;
					y0 =  (locations.shank(2, i) +  locations.lhm(2, i) )/2;

					result = [result; sqrt(  ( x0 - com(1, i) )^2  + ( y0 - com(2, i) )^2 )];
					time = [time, time_(i)];
				else
					% result = [result; 1];
				end
			end
		end

		function [result, time] = ssm(obj, com, supports, locations, time_)
			% locations.stab(1:3, 1)
			result = [];
			time = [];
			for i = 1:obj.resolution
				if( supports.stab(i) && supports.shank(i) )
					% shank -> upper
					% stab  -> lower
					time = [time, time_(i)];

					d1 = sqrt(  ( locations.shank(1, i) - com(1, i) )^2  + ( locations.shank(2, i) - com(2, i) )^2 );
					d2 = sqrt(  ( locations.stab(1, i) - com(1, i) )^2  + ( locations.stab(2, i) - com(2, i) )^2 );
					if(d1<d2)
						result = [result; d1];
					else
						result = [result; d2];
					end

				elseif( supports.stab(i) && supports.lhm(i) )
					% shank -> upper
					% stab  -> lower
					d1 = sqrt(  ( locations.lhm(1, i) - com(1, i) )^2  + ( locations.lhm(2, i) - com(2, i) )^2 );
					d2 = sqrt(  ( locations.stab(1, i) - com(1, i) )^2  + ( locations.stab(2, i) - com(2, i) )^2 );
					time = [time, time_(i)];

					if(d1<d2)
						result = [result; d1];
					else
						result = [result; d2];
					end

				elseif( supports.shank(i) && supports.lhm(i) )
					% shank -> upper
					% stab  -> lower
					d1 = sqrt(  ( locations.lhm(1, i) - com(1, i) )^2  + ( locations.lhm(2, i) - com(2, i) )^2 );
					d2 = sqrt(  ( locations.shank(1, i) - com(1, i) )^2  + ( locations.shank(2, i) - com(2, i) )^2 );
					time = [time, time_(i)];

					if(d1<d2)
						result = [result; d1];
					else
						result = [result; d2];
					end

				end
			end
			% size(result)
			% note com position for all time
			% note supports
			% with supports, calculate minimum distance
		end

		function plot_contacts(obj, contact, time)
			hold off
			subplot(3,1,1), title('Contact switch 1 = on, 0 = off')
			plot(time, contact.stab), xlabel('time (s)');ylabel('stabilizer'), ylim([-0.1, 1.1])
			subplot(3,1,2);
			plot(time, contact.shank), xlabel('time (s)');ylabel('shank'),ylim([-0.1, 1.1])
			subplot(3,1,3);
			plot(time, contact.lhm), xlabel('time (s)');ylabel('LHM'), ylim([-0.1, 1.1])
			hold off
		end

		function obj = animate(obj, pose)
			% [pose, stability] = obj.get_coordinates;
			title('Aerobot analytics','FontSize',12);
			f = figure(1), hold on, grid on, xlabel('X-axis');ylabel('Y-axis');zlabel('Z-axis');

			axis([0 3 -1 1 0 1.])
			az = 180;
			el = 0;
			view(az, el);
			h = zeros(1, 16);
			fields = fieldnames(pose);




            mass = [
                obj.camera_mass;
				obj.shank_mass;
                obj.stab_wheel_mass;
                obj.stab_mass;
                obj.thigh_mass;
                obj.torso_mass;
                obj.lhm_mass;
                obj.left_lhm_wheel_mass;
                obj.right_lhm_wheel_mass;
                obj.left_shoulder_mass;
                obj.right_shoulder_mass;
                obj.left_arm_mass;
                obj.right_arm_mass;
                obj.shank_left_wheel_mass;
                obj.shank_right_wheel_mass;
                ];


            % unproportional 
			for i = 1:15
				pose_ = pose.(fields{i})(1, :);
				h(i) = plot3(pose_(1), pose_(2), pose_(3),'b.','MarkerSize', 39);
			end

			h(16) = plot3(pose.com(1,1),pose.com(2,1),pose.com(3,1),'k.','MarkerSize', 60);

			%  proportional 
			% for i = 1:15
			% 	pose_ = pose.(fields{i})(1, :);
			% 	h(i) = plot3(pose_(1), pose_(2), pose_(3),'b.','MarkerSize', 5*mass(i));
			% end

   %          total_mass = sum(mass);

			% h(16) = plot3(pose.com(1,1),pose.com(2,1),pose.com(3,1),'k.','MarkerSize', 5*total_mass);


			% a = obj.times
			% drawnow;
			% % Playback data at certain frequency

			for j = 1:obj.resolution
				for i = 1:15
					pose_ = pose.(fields{i})(j, :);
		            set(h(i), 'XData', pose_(1));
		            set(h(i), 'YData', pose_(2));
		            set(h(i), 'ZData', pose_(3));
				end
				% plot robot com
				pose_ = pose.('com')(1:3, j);
	            set(h(16), 'XData', pose_(1));
	            set(h(16), 'YData', pose_(2));
	            set(h(16), 'ZData', pose_(3));

	            % pause(0.01);
	            drawnow;
			end

			hold off;


		end

		function [mats, contact, time] = load_gazebo_data(obj, topic)

			[msg, t] = obj.rosbag.readAll(topic);

			time = cellfun(@(x) x.time.time, t);

			% get contacts

			temp = [msg{:}];

			contact_ = [];
			contact_ = [contact_, temp.contact];
			contact.stab = [];
			contact.stab = [contact.stab, contact_.stabilizer];
			contact.lhm = [];
			contact.lhm = [contact.lhm, contact_.lhm];
			contact.shank = [];
			contact.shank = [contact.shank, contact_.shank];
			
			obj.resolution = length(contact.shank);

			% get gazebo joint positions and orientations

			shank_position_accessor = @(msg) msg.shank_footprint.position;
			shank_orientation_accessor = @(msg) msg.shank_footprint.orientation;
			
			thigh_position_accessor = @(msg) msg.thigh_link.position;
			thigh_orientation_accessor = @(msg) msg.thigh_link.orientation;

			torso_position_accessor = @(msg) msg.torso_link.position;
			torso_orientation_accessor = @(msg) msg.torso_link.orientation;
			
			lhm_link_position_accessor = @(msg) msg.lhm_link.position;
			lhm_link_orientation_accessor = @(msg) msg.lhm_link.orientation;

			lhm_wheel_left_link_position_accessor = @(msg) msg.lhm_wheel_left_link.position;
			lhm_wheel_left_link_orientation_accessor = @(msg) msg.lhm_wheel_left_link.orientation;

			lhm_wheel_right_link_position_accessor = @(msg) msg.lhm_wheel_right_link.position;
			lhm_wheel_right_link_orientation_accessor = @(msg) msg.lhm_wheel_right_link.orientation;

			wheel_left_link_position_accessor = @(msg) msg.wheel_left_link.position;
			wheel_left_link_orientation_accessor = @(msg) msg.wheel_left_link.orientation;

			wheel_right_link_position_accessor = @(msg) msg.wheel_right_link.position;
			wheel_right_link_orientation_accessor = @(msg) msg.wheel_right_link.orientation;

			arm_left_link_position_accessor = @(msg) msg.arm_left_link.position;
			arm_left_link_orientation_accessor = @(msg) msg.arm_left_link.orientation;

			arm_right_link_position_accessor = @(msg) msg.arm_right_link.position;
			arm_right_link_orientation_accessor = @(msg) msg.arm_right_link.orientation;

			shoulder_left_link_position_accessor = @(msg) msg.shoulder_left_link.position;
			shoulder_left_link_orientation_accessor = @(msg) msg.shoulder_left_link.orientation;

			shoulder_right_link_position_accessor = @(msg) msg.shoulder_right_link.position;
			shoulder_right_link_orientation_accessor = @(msg) msg.shoulder_right_link.orientation;
			
			stab_link_position_accessor = @(msg) msg.stab_link.position;
			stab_link_orientation_accessor = @(msg) msg.stab_link.orientation;

			stab_wheel_position_accessor = @(msg) msg.stab_wheel.position;
			stab_wheel_orientation_accessor = @(msg) msg.stab_wheel.orientation;

			mats.shank_footprint.position = ros.msgs2mat(msg, shank_position_accessor);
			mats.shank_footprint.orientation = ros.msgs2mat(msg, shank_orientation_accessor);

			mats.thigh_link.position = ros.msgs2mat(msg, thigh_position_accessor);
			mats.thigh_link.orientation = ros.msgs2mat(msg, thigh_orientation_accessor);


			mats.torso_link.position = ros.msgs2mat(msg, torso_position_accessor);
			mats.torso_link.orientation = ros.msgs2mat(msg, torso_orientation_accessor);

			mats.lhm_link.position = ros.msgs2mat(msg, lhm_link_position_accessor);
			mats.lhm_link.orientation = ros.msgs2mat(msg, lhm_link_orientation_accessor);


			mats.lhm_wheel_left_link.position = ros.msgs2mat(msg, lhm_wheel_left_link_position_accessor);
			mats.lhm_wheel_left_link.orientation = ros.msgs2mat(msg, lhm_wheel_left_link_orientation_accessor);

			mats.lhm_wheel_right_link.position = ros.msgs2mat(msg, lhm_wheel_right_link_position_accessor);
			mats.lhm_wheel_right_link.orientation = ros.msgs2mat(msg, lhm_wheel_right_link_orientation_accessor);


			mats.wheel_left_link.position = ros.msgs2mat(msg, wheel_left_link_position_accessor);
			mats.wheel_left_link.orientation = ros.msgs2mat(msg, wheel_left_link_orientation_accessor);


			mats.wheel_right_link.position = ros.msgs2mat(msg, wheel_right_link_position_accessor);
			mats.wheel_right_link.orientation = ros.msgs2mat(msg, wheel_right_link_orientation_accessor);
			

			mats.arm_left_link.position = ros.msgs2mat(msg, arm_left_link_position_accessor);
			mats.arm_left_link.orientation = ros.msgs2mat(msg, arm_left_link_orientation_accessor);

			mats.arm_right_link.position = ros.msgs2mat(msg, arm_right_link_position_accessor);
			mats.arm_right_link.orientation = ros.msgs2mat(msg, arm_right_link_orientation_accessor);


			mats.wheel_right_link.position = ros.msgs2mat(msg, wheel_right_link_position_accessor);
			mats.wheel_right_link.orientation = ros.msgs2mat(msg, wheel_right_link_orientation_accessor);


			mats.shoulder_left_link.position = ros.msgs2mat(msg, shoulder_left_link_position_accessor);
			mats.shoulder_left_link.orientation = ros.msgs2mat(msg, shoulder_left_link_orientation_accessor);
			
			mats.shoulder_right_link.position = ros.msgs2mat(msg, shoulder_right_link_position_accessor);
			mats.shoulder_right_link.orientation = ros.msgs2mat(msg, shoulder_right_link_orientation_accessor);


			mats.wheel_right_link.position = ros.msgs2mat(msg, wheel_right_link_position_accessor);
			mats.wheel_right_link.orientation = ros.msgs2mat(msg, wheel_right_link_orientation_accessor);

			mats.stab_link.position = ros.msgs2mat(msg, stab_link_position_accessor);
			mats.stab_link.orientation = ros.msgs2mat(msg, stab_link_orientation_accessor);

			mats.stab_wheel.position = ros.msgs2mat(msg, stab_wheel_position_accessor);
			mats.stab_wheel.orientation = ros.msgs2mat(msg, stab_wheel_orientation_accessor);

		end

	end

end

			% Plot full history
			% plot3(pose.camera_com(:,1),pose.camera_com(:,2),pose.camera_com(:,3),'b.','MarkerSize', 39), grid on, hold on
			% plot3(pose.shank_footprint_com(:,1),pose.shank_footprint_com(:,2),pose.shank_footprint_com(:,3),'b.','MarkerSize', 39), 
			% plot3(pose.stab_wheel_com(:,1),pose.stab_wheel_com(:,2),pose.stab_wheel_com(:,3),'b.','MarkerSize', 39), 
			% plot3(pose.thigh_link_com(:,1),pose.thigh_link_com(:,2),pose.thigh_link_com(:,3),'b.','MarkerSize', 39), 
			% plot3(pose.torso_link_com(:,1),pose.torso_link_com(:,2),pose.torso_link_com(:,3),'b.','MarkerSize', 39), 
			% plot3(pose.lhm_link_com(:,1),pose.lhm_link_com(:,2),pose.lhm_link_com(:,3),'b.','MarkerSize', 39), 
			% plot3(pose.lhm_wheel_left_link_com(:,1),pose.lhm_wheel_left_link_com(:,2),pose.lhm_wheel_left_link_com(:,3),'b.','MarkerSize', 39), 
			% plot3(pose.lhm_wheel_right_link_com(:,1),pose.lhm_wheel_right_link_com(:,2),pose.lhm_wheel_right_link_com(:,3),'b.','MarkerSize', 39), 
			% plot3(pose.shoulder_left_link_com(:,1),pose.shoulder_left_link_com(:,2),pose.shoulder_left_link_com(:,3),'b.','MarkerSize', 39), 
			% plot3(pose.shoulder_right_link_com(:,1),pose.shoulder_right_link_com(:,2),pose.shoulder_right_link_com(:,3),'b.','MarkerSize', 39), 
			% plot3(pose.arm_left_link_com(:,1),pose.arm_left_link_com(:,2),pose.arm_left_link_com(:,3),'b.','MarkerSize', 39), 
			% plot3(pose.arm_right_link_com(:,1),pose.arm_right_link_com(:,2),pose.arm_right_link_com(:,3),'b.','MarkerSize', 39), 
			% plot3(pose.wheel_left_link_com (:,1),pose.wheel_left_link_com (:,2),pose.wheel_left_link_com (:,3),'b.','MarkerSize', 39), 
			% plot3(pose.wheel_right_link_com(:,1),pose.wheel_right_link_com(:,2),pose.wheel_right_link_com(:,3),'b.','MarkerSize', 39), 
			% xlabel('X-axis');ylabel('Y-axis');zlabel('Z-axis');



			% plot3(pose.camera_com(:,1),pose.camera_com(:,2),pose.camera_com(:,3),'k-','MarkerSize', 39), grid on, hold on
			% plot3(pose.shank_footprint_com(:,1),pose.shank_footprint_com(:,2),pose.shank_footprint_com(:,3),'k-');
			% plot3(pose.stab_wheel_com(:,1),pose.stab_wheel_com(:,2),pose.stab_wheel_com(:,3),'k-');
			% plot3(pose.thigh_link_com(:,1),pose.thigh_link_com(:,2),pose.thigh_link_com(:,3),'k-');
			% plot3(pose.torso_link_com(:,1),pose.torso_link_com(:,2),pose.torso_link_com(:,3),'k-');
			% plot3(pose.lhm_link_com(:,1),pose.lhm_link_com(:,2),pose.lhm_link_com(:,3),'k-');
			% plot3(pose.lhm_wheel_left_link_com(:,1),pose.lhm_wheel_left_link_com(:,2),pose.lhm_wheel_left_link_com(:,3),'k-');
			% plot3(pose.lhm_wheel_right_link_com(:,1),pose.lhm_wheel_right_link_com(:,2),pose.lhm_wheel_right_link_com(:,3),'k-'); 
			% plot3(pose.shoulder_left_link_com(:,1),pose.shoulder_left_link_com(:,2),pose.shoulder_left_link_com(:,3),'k-','MarkerSize', 39); 
			% plot3(pose.shoulder_right_link_com(:,1),pose.shoulder_right_link_com(:,2),pose.shoulder_right_link_com(:,3),'k-','MarkerSize', 39); 
			% plot3(pose.arm_left_link_com(:,1),pose.arm_left_link_com(:,2),pose.arm_left_link_com(:,3),'k-','MarkerSize', 39);
			% plot3(pose.arm_right_link_com(:,1),pose.arm_right_link_com(:,2),pose.arm_right_link_com(:,3),'k-','MarkerSize', 39); 
			% plot3(pose.wheel_left_link_com (:,1),pose.wheel_left_link_com (:,2),pose.wheel_left_link_com (:,3),'k-','MarkerSize', 39); 
			% plot3(pose.wheel_right_link_com(:,1),pose.wheel_right_link_com(:,2),pose.wheel_right_link_com(:,3),'k-','MarkerSize', 39); 


			% h(1) = plot3(pose.camera_com(1,1),pose.camera_com(1,2),pose.camera_com(1,3),'b.','MarkerSize', 39);
			% h(2) = plot3(pose.shank_footprint_com(1,1),pose.shank_footprint_com(1,2),pose.shank_footprint_com(1,3),'b.','MarkerSize', 39);
			% h(3) = plot3(pose.stab_wheel_com(1,1),pose.stab_wheel_com(1,2),pose.stab_wheel_com(1,3),'b.','MarkerSize', 39);
			% h(4) = plot3(pose.thigh_link_com(1,1),pose.thigh_link_com(1,2),pose.thigh_link_com(1,3),'b.','MarkerSize', 39);
			% h(5) = plot3(pose.torso_link_com(1,1),pose.torso_link_com(1,2),pose.torso_link_com(1,3),'b.','MarkerSize', 39); 
			% h(6) = plot3(pose.lhm_link_com(1,1),pose.lhm_link_com(1,2),pose.lhm_link_com(1,3),'b.','MarkerSize', 39); 
			% h(7) = plot3(pose.lhm_wheel_left_link_com(1,1),pose.lhm_wheel_left_link_com(1,2),pose.lhm_wheel_left_link_com(1,3),'b.','MarkerSize', 39);
			% h(8) = plot3(pose.lhm_wheel_right_link_com(1,1),pose.lhm_wheel_right_link_com(1,2),pose.lhm_wheel_right_link_com(1,3),'b.','MarkerSize', 39);
			% h(9) = plot3(pose.shoulder_left_link_com(1,1),pose.shoulder_left_link_com(1,2),pose.shoulder_left_link_com(1,3),'b.','MarkerSize', 39); 
			% h(10) = plot3(pose.shoulder_right_link_com(1,1),pose.shoulder_right_link_com(1,2),pose.shoulder_right_link_com(1,3),'b.','MarkerSize', 39);
			% h(11) = plot3(pose.arm_left_link_com(1,1),pose.arm_left_link_com(1,2),pose.arm_left_link_com(1,3),'b.','MarkerSize', 39);
			% h(12) = plot3(pose.arm_right_link_com(1,1),pose.arm_right_link_com(1,2),pose.arm_right_link_com(1,3),'b.','MarkerSize', 39); 
			% h(13) = plot3(pose.wheel_left_link_com (1,1),pose.wheel_left_link_com (1,2),pose.wheel_left_link_com (1,3),'b.','MarkerSize', 39);
			% h(14) = plot3(pose.wheel_right_link_com(1,1),pose.wheel_right_link_com(1,2),pose.wheel_right_link_com(1,3),'b.','MarkerSize', 39);
			% h(15) = plot3(pose.stab_link_com(1,1),pose.stab_link_com(1,2),pose.stab_link_com(1,3),'b.','MarkerSize', 39);


		% function [positions, time] = plot_joint_error_positions(obj, bag_name)
		% 	topic = '/robbie/whole_body_controller/state';
		% 	[pos, t] = obj.rosbag.readAll(topic);
		% 	joints = zeros(8, length(pos));
		% 	time = [];
		% 	for i = 1:length(pos)
		% 		joints(1, i) = pos{i}.error.positions(1);
		% 		joints(2, i) = pos{i}.error.positions(2);
		% 		joints(3, i) = pos{i}.error.positions(3);
		% 		joints(4, i) = pos{i}.error.positions(4);
		% 		joints(5, i) = pos{i}.error.positions(5);
		% 		joints(6, i) = pos{i}.error.positions(6);
		% 		joints(7, i) = pos{i}.error.positions(7);
		% 		joints(8, i) = pos{i}.error.positions(8);
		% 		time = [time; t{i}.time.time];
		% 	end

		% 	figure, plot(time, joints), grid on, title('Joint position error vs Time'), xlabel('time (s)'), ylabel('joint position error'), legend('stabilser','thigh','hip','lhm','left shoulder','right shoulder','left elbow','right elbow');
			
		% 	hold off

		% 	% color = {'k^','bh','r:','g','p','c','y','b+'} ;
		% 	% figure
		% 	% for i = 1:8
		% 	% 	plot(time, joints(i, :),color{i},'MarkerSize', 5), hold on,  grid on;
		% 	% end
		% 	% xlabel('time (s)'), ylabel('joint position'), legend('stabilser','thigh','hip','lhm','left shoulder','right shoulder','left elbow','right elbow');
		% end

		% function [velocities, time] = plot_joint_velocities(obj, bag_name)
		% 	topic = '/robbie/whole_body_controller/state';
		% 	[pos, t] = obj.rosbag.readAll(topic);
		% 	joints = zeros(8, length(pos));
		% 	time = [];
		% 	for i = 1:length(pos)
		% 		joints(1, i) = pos{i}.actual.velocities(1);
		% 		joints(2, i) = pos{i}.actual.velocities(2);
		% 		joints(3, i) = pos{i}.actual.velocities(3);
		% 		joints(4, i) = pos{i}.actual.velocities(4);
		% 		joints(5, i) = pos{i}.actual.velocities(5);
		% 		joints(6, i) = pos{i}.actual.velocities(6);
		% 		joints(7, i) = pos{i}.actual.velocities(7);
		% 		joints(8, i) = pos{i}.actual.velocities(8);
		% 		time = [time; t{i}.time.time];
		% 	end

		% 	figure, plot(time, joints), grid on, title('Joint velocities vs Time'), xlabel('time (s)'), ylabel('joint velocities'), legend('stabilser','thigh','hip','lhm','left shoulder','right shoulder','left elbow','right elbow');
			
		% 	hold off

		% 	% color = {'k^','bh','r:','g','p','c','y','b+'} ;
		% 	% figure
		% 	% for i = 1:8
		% 	% 	plot(time, joints(i, :),color{i},'MarkerSize', 5), hold on,  grid on;
		% 	% end
		% 	% xlabel('time (s)'), ylabel('joint position'), legend('stabilser','thigh','hip','lhm','left shoulder','right shoulder','left elbow','right elbow');
		% end

		% function [velocities, time] = plot_joint_accelerations(obj, bag_name)
		% 	topic = '/robbie/whole_body_controller/state';
		% 	[pos, t] = obj.rosbag.readAll(topic);
		% 	joints = zeros(8, length(pos));
		% 	time = [];
		% 	for i = 1:length(pos)
		% 		joints(1, i) = pos{i}.actual.accelerations(1);
		% 		joints(2, i) = pos{i}.actual.accelerations(2);
		% 		joints(3, i) = pos{i}.actual.accelerations(3);
		% 		joints(4, i) = pos{i}.actual.accelerations(4);
		% 		joints(5, i) = pos{i}.actual.accelerations(5);
		% 		joints(6, i) = pos{i}.actual.accelerations(6);
		% 		joints(7, i) = pos{i}.actual.accelerations(7);
		% 		joints(8, i) = pos{i}.actual.accelerations(8);
		% 		time = [time; t{i}.time.time];
		% 	end

		% 	figure, plot(time, joints), grid on, title('Joint accelerations vs Time'), xlabel('time (s)'), ylabel('joint accelerations'), legend('stabilser','thigh','hip','lhm','left shoulder','right shoulder','left elbow','right elbow');
			
		% 	hold off

		% 	% color = {'k^','bh','r:','g','p','c','y','b+'} ;
		% 	% figure
		% 	% for i = 1:8
		% 	% 	plot(time, joints(i, :),color{i},'MarkerSize', 5), hold on,  grid on;
		% 	% end
		% 	% xlabel('time (s)'), ylabel('joint position'), legend('stabilser','thigh','hip','lhm','left shoulder','right shoulder','left elbow','right elbow');
		% end


			% _position_accessor = @(msg) msg.thigh_link.position;
			% _orientation_accessor = @(msg) msg.thigh_link.orientation;

			% mats._link.position = ros.msgs2mat(msg, _position_accessor);
			% mats._link.orientation = ros.msgs2mat(msg, _orientation_accessor);




			% mats.shank_footprint = shank_footprint;





			% shank_position_accessor = @(msg) msg.states.states.pose(3).position;
			% shank_orientation_accessor = @(msg) msg.states.states.pose(3).orientation;
			% thigh_position_accessor = @(msg) msg.states.states.pose(4).position;
			% thigh_orientation_accessor = @(msg) msg.states.states.pose(4).orientation;
			
			% [shank_footprint_position] = ros.msgs2mat(msg, shank_position_accessor); 
			% [shank_footprint_orientation] = ros.msgs2mat(msg, shank_orientation_accessor); 
			% [thigh_position] = ros.msgs2mat(msg, thigh_position_accessor); 
			% [thigh_orientation] = ros.msgs2mat(msg, thigh_orientation_accessor); 
			
			% mats.shank_footprint.position = shank_footprint_position;
			% mats.shank_footprint.orientation = shank_footprint_orientation;

			% mats.thigh_link.position = thigh_position;
			% mats.thigh_link.orientation = thigh_orientation;
			% thigh_link_accessor = @(msg) msg.states.states.pose(4).position;
			% [shank_footprint] = ros.msgs2mat(msg, _accessor); 
			

			% mats =
			% for i = 1:length(msg)
			% 	positions = [positions; [msg{i}.states.states.pose.position]'];
			% 	orientations = [orientations; [msg{i}.states.states.pose.orientation]'];
			% end
			% mats.positions = positions;
			% mats.msg = msg;
			% mats.orientations = orientations;
			% disp(msg);
			% a = msg{:};
			% b = [a.states];
			% c = [b.states];
			% d = [c.states];
			% b = [a.states]
			% mats.shank_footprint = [mats.shank_footprint, msg{}]
			% mats.pos = pos;
			% mats.t = t;
			% times = linspace(tree.time_begin + 1, tree.time_end - 1, obj.resolution);
			% Load tf data

			% mats.camera_link = tree.lookup(world_frame, 'camera_link', times);
			% mats.shank_footprint = tree.lookup(world_frame, 'shank_footprint', times);
			% mats.shank_link = tree.lookup(world_frame, 'shank_link', times);
			% mats.thigh_link = tree.lookup(world_frame, 'thigh_link', times);
			% mats.torso_link = tree.lookup(world_frame, 'torso_link', times);
			% mats.lhm_link = tree.lookup(world_frame, 'lhm_link', times);
			% mats.lhm_wheel_right_link = tree.lookup(world_frame, 'lhm_wheel_right_link', times);
			% mats.lhm_wheel_left_link = tree.lookup(world_frame, 'lhm_wheel_left_link', times);
			% mats.shoulder_left_link = tree.lookup(world_frame, 'shoulder_left_link', times);
			% mats.shoulder_right_link = tree.lookup(world_frame, 'shoulder_right_link', times);
			% mats.arm_right_link = tree.lookup(world_frame, 'arm_right_link', times);
			% mats.arm_left_link = tree.lookup(world_frame, 'arm_left_link', times);
			% mats.wheel_right_link = tree.lookup(world_frame, 'wheel_right_link', times);
			% mats.wheel_left_link = tree.lookup(world_frame, 'wheel_left_link', times);
			% mats.stab_link = tree.lookup(world_frame, 'stab_link', times);
			% mats.stabwheel = tree.lookup(world_frame, 'stab_wheel', times);

