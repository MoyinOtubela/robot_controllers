classdef aerobot_analysis
	properties
		rosbag_dir = '/home/moyin/dev/autonomous_controllers/src/robot_controllers/robbie_control/src/'
		rosbag;

		world_frame = 'odom';

        shank_mass = 7;
        stab_mass = 0.04625;
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

        resolution = 100;

	end

	methods

		function obj = aerobot_analysis(bag_name, resolution)
			close all;
			addpath('/home/moyin/dev/autonomous_controllers/src/matlab_rosbag');
			% obj.times = zeros(1, resolution);
			obj.resolution = resolution;
			% set up mass
			obj.rosbag = ros.Bag.load(strcat(obj.rosbag_dir,bag_name,'.active'));
			disp(obj.rosbag.info());
		end



		function [joints, time] = plot_joint_positions(obj, bag_name)
			topic = '/robbie/whole_body_controller/state';
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

			figure, plot(time, joints), grid on, title('Joint positions vs Time'), xlabel('time (s)'), ylabel('joint position'), legend('stabilser','thigh','hip','lhm','left shoulder','right shoulder','left elbow','right elbow');
			
			hold off

			% color = {'k^','bh','r:','g','p','c','y','b+'} ;
			% figure
			% for i = 1:8
			% 	plot(time, joints(i, :),color{i},'MarkerSize', 5), hold on,  grid on;
			% end
			% xlabel('time (s)'), ylabel('joint position'), legend('stabilser','thigh','hip','lhm','left shoulder','right shoulder','left elbow','right elbow');
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
			for i = m
				mat =  ones(4,4);
				mat(1:3, 1:3) = obj.get_rotation_matrix([i.rotation]); 
				mat(1:3,4) = [i.translation];
			    coord =  mat*point;
			    pos = [pos;coord(1:3)'];
			end

		end

		function result = ssm_delta(obj, com, supports, locations)
			result = [];
			for i = 1:obj.resolution

				if( supports.lhm(i) && supports.stab(i) )
					x0 =  (locations.stab(1, i) +  locations.lhm(1, i) )/2;
					y0 =  (locations.stab(2, i) +  locations.lhm(2, i) )/2;

					result = [result; sqrt(  ( x0 - com(1, i) )^2  + ( y0 - com(2, i) )^2 )];
				elseif( supports.stab(i) && supports.shank(i) )
					% shank -> upper
					% stab  -> lower
					x0 =  (locations.shank(1, i) +  locations.stab(1, i) )/2;
					y0 =  (locations.shank(2, i) +  locations.stab(2, i) )/2;

					result = [result; sqrt(  ( x0 - com(1, i) )^2  + ( y0 - com(2, i) )^2 )];
				elseif( supports.lhm(i) && supports.shank(i) )
					x0 =  (locations.shank(1, i) +  locations.lhm(1, i) )/2;
					y0 =  (locations.shank(2, i) +  locations.lhm(2, i) )/2;

					result = [result; sqrt(  ( x0 - com(1, i) )^2  + ( y0 - com(2, i) )^2 )];
				else
					result = [result; 1];
				end
			end
		end

		function result = ssm(obj, com, supports, locations)
			% locations.stab(1:3, 1)
			result = [];
			for i = 1:obj.resolution
				if( supports.stab(i) && supports.shank(i) )
					% shank -> upper
					% stab  -> lower
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

		function stability = analyse_stability(obj, com, supports, locations)
			% stability = 0;

			stability.ssm = obj.ssm(com, supports, locations);
			stability.ssm_delta = obj.ssm_delta(com, supports, locations);

		end

		function contact = determine_contact(obj, support, wheel_radius)
			contact = [];
			for i = [support.translation]
				data = abs(i(3) - wheel_radius);
				condition = data <= 0.01;
				contact = [contact; condition];
			end

		end

		function [pose, contacts, locations] = get_coordinates(obj, mats)

			% mats = obj.load_tree;
			pose.camera_com = obj.transform_point(mats.camera_link, [0 0 0 1]');
			pose.shank_footprint_com = obj.transform_point(mats.shank_footprint, [0.163037 0 0.159341 1]');
			pose.stab_wheel_com = obj.transform_point(mats.stabwheel, [0 0 0 1]');
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

			pose.com = zeros(3, obj.resolution);

			fields;

			for j = 1:obj.resolution
				for i = 1:15
					pose_ = pose.(fields{i})(j, :);
					pose.com(1, j) = pose_(1)*mass(i) + pose.com(1, j);
					pose.com(2, j) = pose_(2)*mass(i) + pose.com(2, j);
					pose.com(3, j) = pose_(3)*mass(i) + pose.com(3, j);
				end
				pose.com(1, j) = pose.com(1, j)/total_mass;
				pose.com(2, j) = pose.com(2, j)/total_mass;
				pose.com(3, j) = pose.com(3, j)/total_mass;
			end

			contacts.shank = obj.determine_contact(mats.shank_link, obj.shank_wheel_radius);
			contacts.stab = obj.determine_contact(mats.stabwheel, obj.stab_wheel_radius);
			contacts.lhm = obj.determine_contact(mats.lhm_wheel_left_link, obj.lhm_wheel_radius);
			% pose.com(1:3, 1)

			locations.shank = [mats.shank_footprint.translation];
			locations.stab = [mats.stabwheel.translation];
			locations.lhm = [mats.lhm_link.translation];

			% stability = obj.analyse_stability(pose.com, supports, locations);

			% disp(stability.ssm)
			% % times = linspace(obj.tree.time_begin + 1, obj.tree.time_end - 1, obj.resolution);
			% fig = figure(2)


		end


		function obj = animate(obj, pose, stability)
			% [pose, stability] = obj.get_coordinates;
			title('Aerobot analytics','FontSize',12);
			f = figure(1), hold on, grid on, xlabel('X-axis');ylabel('Y-axis');zlabel('Z-axis');

			axis([-1 1 -1 1 0 1])
			az = 180;
			el = 0;
			view(az, el);


			h = zeros(1, 15);
			fields = fieldnames(pose);

			for i = 1:15
				pose_ = pose.(fields{i})(1, :);
				h(i) = plot3(pose_(1), pose_(2), pose_(3),'b.','MarkerSize', 39);
			end

			h(16) = plot3(pose.com(1,1),pose.com(2,1),pose.com(3,1),'k.','MarkerSize', 60);


   %          mass = [
   %              obj.camera_mass;
			% 	obj.shank_mass;
   %              obj.stab_wheel_mass;
   %              obj.stab_mass;
   %              obj.thigh_mass;
   %              obj.torso_mass;
   %              obj.lhm_mass;
   %              obj.left_lhm_wheel_mass;
   %              obj.right_lhm_wheel_mass;
   %              obj.left_shoulder_mass;
   %              obj.right_shoulder_mass;
   %              obj.left_arm_mass;
   %              obj.right_arm_mass;
   %              obj.shank_left_wheel_mass;
   %              obj.shank_right_wheel_mass;
   %              ];


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

	            % pause(s);
	            drawnow;
			end

			hold off;

			figure, plot(1:obj.resolution, stability.ssm), title('Static stability analysis','FontSize',12), hold on, grid on, axis([0 1000 -0.1 0.3])

			plot(1:obj.resolution, stability.ssm_delta), legend('ssm','ssm\_delta'), hold off

		end

		function mats = load_tree(obj)
			world_frame = obj.world_frame;
			tree = ros.TFTree(obj.rosbag);
			disp(tree.allFrames());
			times = linspace(tree.time_begin + 1, tree.time_end - 1, obj.resolution);
			% Load tf data
			mats.camera_link = tree.lookup(world_frame, 'camera_link', times);
			mats.shank_footprint = tree.lookup(world_frame, 'shank_footprint', times);
			mats.shank_link = tree.lookup(world_frame, 'shank_link', times);
			mats.thigh_link = tree.lookup(world_frame, 'thigh_link', times);
			mats.torso_link = tree.lookup(world_frame, 'torso_link', times);
			mats.lhm_link = tree.lookup(world_frame, 'lhm_link', times);
			mats.lhm_wheel_right_link = tree.lookup(world_frame, 'lhm_wheel_right_link', times);
			mats.lhm_wheel_left_link = tree.lookup(world_frame, 'lhm_wheel_left_link', times);
			mats.shoulder_left_link = tree.lookup(world_frame, 'shoulder_left_link', times);
			mats.shoulder_right_link = tree.lookup(world_frame, 'shoulder_right_link', times);
			mats.arm_right_link = tree.lookup(world_frame, 'arm_right_link', times);
			mats.arm_left_link = tree.lookup(world_frame, 'arm_left_link', times);
			mats.wheel_right_link = tree.lookup(world_frame, 'wheel_right_link', times);
			mats.wheel_left_link = tree.lookup(world_frame, 'wheel_left_link', times);
			mats.stab_link = tree.lookup(world_frame, 'stab_link', times);
			mats.stabwheel = tree.lookup(world_frame, 'stab_wheel', times);

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
