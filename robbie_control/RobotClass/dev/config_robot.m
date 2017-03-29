classdef config_robot < handle
	properties(Hidden = true)
		h0;
		h1;
		h2;
		h3;
		h4;
		h5;
		h6;
		h7;
		h8;
		h9;
		h10;
		h11;
		h12;
		h13;
		h14;
		h15;
		h16;
		h17;
        h18;
        h19;
		h_com;


        a_0;    alpha_0;    d_0;    theta_0;
        a_1;    alpha_1;    d_1;    theta_1;
        a_2;    alpha_2;    d_2;    theta_2;
        a_3;    alpha_3;    d_3;    theta_3;
        a_4;    alpha_4;    d_4;    theta_4;
        a_5;    alpha_5;    d_5;    theta_5;
        a_6;    alpha_6;    d_6;    theta_6;
        a_7;    alpha_7;    d_7;    theta_7;
        a_8;    alpha_8;    d_8;    theta_8;
        a_9;    alpha_9;    d_9;    theta_9;
        a_10;   alpha_10;   d_10;   theta_10;
        a_11;   alpha_11;   d_11;   theta_11;
        a_12;   alpha_12;   d_12;   theta_12;
        a_13;   alpha_13;   d_13;   theta_13;
        a_14;   alpha_14;   d_14;   theta_14;
        a_15;   alpha_15;   d_15;   theta_15;
        a_16;   alpha_16;   d_16;   theta_16;

        a_17;   alpha_17;   d_17;   theta_17;
        a_18;   alpha_18;   d_18;   theta_18;
        % LHM
        a_19;   alpha_19;   d_19;   theta_19;  
        a_20;   alpha_20;   d_20;   theta_20;   
        a_21;   alpha_21;   d_21;   theta_21;   
        a_22;   alpha_22;   d_22;   theta_22;   
        a_23;   alpha_23;   d_23;   theta_23;   
        
        A0; A1; A2; A3; A4; A5; A6; A7; A8; A9; A10; A11; A12; A13;
        A14; A15; A16; A17; A18; A19; A20; A21; A22; A23;

		drive_wheel_rad = 0.100886;
        % stab_wheel_rad = 0.0762;
        stab_wheel_rad = 0.0750555;
		lhm_wheel_rad = 0.102175;

        hip_monitor = 0;

		% LIMB LENGTHS/WIDTHS (m)
		% shank
		shank_len = 0.347;
		shank_width = 0.356/2;
		shank_depth = 0.15;

        shank_to_stab = 0.24;

		% stabiliser 
		stab_len = 0.25;

		% thigh 
		thigh_len = 0.363;

		% torso params
		torso_len = 0.384;
		torso_width = 0.36;
		bicep_len = 0.3;
		arm_ext = 0.16;

		% LHM params
		lhm_width = 0.425/2;
		% lhm_elongation = 0.2; % aka d_lhm
		lhm_elongation = 0; % aka d_lhm
		lhm_hip_offset = 0.336; % aka k_lhm

		% JOINT VALUES (degrees)
		shank_angle;
		knee_angle;
		hip_angle;
		r_shoulder_angle = 0; % 
		l_shoulder_angle = 0; % 
		stab_angle;
        left_arm_ext = -0.2;
        right_arm_ext = -0.2;
        lhm_position = 0;

		shank_footprint;

        shank_mass = 7;
        % stab_mass = 0.04625;
        stab_mass = 1;
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

        mass = [];
        total_mass;

        stab_height = 0;


	end

	methods(Abstract, Hidden = true)
		updateJoints(obj);
		update(obj);
	end

	% methods(Abstract, Access = protected)
	% 	update(obj);
	% end


	methods(Access = protected)
        function obj = setupDH(obj)
            obj.A0 = obj.DH(obj.a_0, obj.alpha_0, obj.d_0, obj.theta_0);         
            obj.A1 = obj.DH(obj.a_1, obj.alpha_1, obj.d_1, obj.theta_1);         
            obj.A2 = obj.DH(obj.a_2, obj.alpha_2, obj.d_2, obj.theta_2);         
            obj.A3 = obj.DH(obj.a_3, obj.alpha_3, obj.d_3, obj.theta_3);
            obj.A4 = obj.DH(obj.a_4, obj.alpha_4, obj.d_4, obj.theta_4);
            obj.A5 = obj.DH(obj.a_5, obj.alpha_5, obj.d_5, obj.theta_5);
            obj.A6 = obj.DH(obj.a_6, obj.alpha_6, obj.d_6, obj.theta_6);
            obj.A7 = obj.DH(obj.a_7, obj.alpha_7, obj.d_7, obj.theta_7);
            obj.A8 = obj.DH(obj.a_8, obj.alpha_8, obj.d_8, obj.theta_8);
            obj.A9 = obj.DH(obj.a_9, obj.alpha_9, obj.d_9, obj.theta_9);
            obj.A10 = obj.DH(obj.a_10, obj.alpha_10, obj.d_10, obj.theta_10);
            obj.A11 = obj.DH(obj.a_11, obj.alpha_11, obj.d_11, obj.theta_11);
            obj.A12 = obj.DH(obj.a_12, obj.alpha_12, obj.d_12, obj.theta_12);
            obj.A13 = obj.DH(obj.a_13, obj.alpha_13, obj.d_13, obj.theta_13);
            obj.A14 = obj.DH(obj.a_14, obj.alpha_14, obj.d_14, obj.theta_14);
            obj.A15 = obj.DH(obj.a_15, obj.alpha_15, obj.d_15, obj.theta_15);
            obj.A16 = obj.DH(obj.a_16, obj.alpha_16, obj.d_16, obj.theta_16);
            obj.A17 = obj.DH(obj.a_17, obj.alpha_17, obj.d_17, obj.theta_17);
            obj.A18 = obj.DH(obj.a_18, obj.alpha_18, obj.d_18, obj.theta_18);
            obj.A19 = obj.DH(obj.a_19, obj.alpha_19, obj.d_19, obj.theta_19);
            obj.A20 = obj.DH(obj.a_20, obj.alpha_20, obj.d_20, obj.theta_20);
            obj.A21 = obj.DH(obj.a_21, obj.alpha_21, obj.d_21, obj.theta_21);
            obj.A22 = obj.DH(obj.a_22, obj.alpha_22, obj.d_22, obj.theta_22);
            obj.A23 = obj.DH(obj.a_23, obj.alpha_23, obj.d_23, obj.theta_23);
        end


		function output = setUpCOMParam(obj, mass, dh)
			output = com_data;
			output.mass = mass;
			output.A = obj.DH(dh(1), dh(2), dh(3), dh(4));

		end
	end

	methods(Access = public)

        function obj = config_robot
            obj.mass = [obj.shank_mass;
                obj.stab_mass;
                obj.thigh_mass;
                obj.torso_mass;
                obj.lhm_mass;
                obj.left_shoulder_mass;
                obj.right_shoulder_mass;
                obj.left_arm_mass;
                obj.right_arm_mass;
                obj.stab_wheel_mass;
                obj.shank_left_wheel_mass;
                obj.shank_right_wheel_mass;
                obj.left_lhm_wheel_mass;
                obj.right_lhm_wheel_mass;
                obj.camera_mass;
                ];

            obj.total_mass = sum(obj.mass);

        end

		function obj = configure(obj, joints)
			% obj.shank_angle = 0;
            % obj.stab_angle = joints(1);
            obj.shank_angle = joints(1);
            obj.knee_angle = joints(2);
			obj.hip_angle = joints(3);
            obj.lhm_position = joints(4);
			obj.l_shoulder_angle = joints(5);
			obj.r_shoulder_angle = joints(6);
            obj.left_arm_ext = joints(7);
            obj.right_arm_ext = joints(8);
            % joints

            % joints(9) = obj.hip_monitor;

            % disp(joints(9))

			k = (pi/2 -  (1.22173048 + joints(1)));

			h = obj.shank_to_stab*sin(k - obj.shank_rotation);
            obj.stab_angle =  pi/2 - (1.22173048 + obj.shank_rotation + joints(1)) + asin((h+(obj.drive_wheel_rad-obj.stab_wheel_rad+obj.shank_height) - obj.stab_height)/(obj.stab_len)) - 0.698131701;
			obj.update;

		end	


		function [x, y, z] = circle3D(obj, center, normal, radius)
			theta = 0:0.01:2*pi;
			v=null(normal);
			points=repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
			x = points(1,:);
			y = points(2,:);
			z = points(3,:);
		end

		function obj = refresh(obj)

            set(obj.h1, 'XData', obj.joint_locations(2:18,1));
            set(obj.h1, 'YData',obj.joint_locations(2:18,2));
            set(obj.h1, 'ZData',obj.joint_locations(2:18,3));

            set(obj.h2, 'XData', obj.joint_locations(2:4,1));
            set(obj.h2, 'YData',obj.joint_locations(2:4,2));
            set(obj.h2,'ZData',obj.joint_locations(2:4,3));

            set(obj.h3, 'XData', [obj.joint_locations(3, 1); obj.joint_locations(5, 1)]);
            set(obj.h3, 'YData',[obj.joint_locations(3, 2); obj.joint_locations(5, 2)]);
            set(obj.h3, 'ZData',[obj.joint_locations(3, 3); obj.joint_locations(5, 3)]);

            set(obj.h4, 'XData', obj.joint_locations(5:7,1));
            set(obj.h4, 'YData',obj.joint_locations(5:7,2));
            set(obj.h4,'ZData',obj.joint_locations(5:7,3));

            set(obj.h5, 'XData', [obj.joint_locations(6, 1); obj.joint_locations(8, 1)]);
            set(obj.h5, 'YData',[obj.joint_locations(6, 2); obj.joint_locations(8, 2)]);
            set(obj.h5,'ZData',[obj.joint_locations(6, 3); obj.joint_locations(8, 3)]);

            set(obj.h6, 'XData', [obj.joint_locations(6, 1); obj.joint_locations(9, 1)]);
            set(obj.h6, 'YData',[obj.joint_locations(6, 2); obj.joint_locations(9, 2)]);
            set(obj.h6, 'ZData',[obj.joint_locations(6, 3); obj.joint_locations(9, 3)]);

            set(obj.h7, 'XData', [obj.joint_locations(8, 1); obj.joint_locations(10, 1)]);
            set(obj.h7, 'YData',[obj.joint_locations(8, 2); obj.joint_locations(10, 2)]);
            set(obj.h7, 'ZData',[obj.joint_locations(8, 3); obj.joint_locations(10, 3)]);

            set(obj.h8, 'XData', [obj.joint_locations(10, 1); obj.joint_locations(12, 1)]);
            set(obj.h8, 'YData',[obj.joint_locations(10, 2); obj.joint_locations(12, 2)]);
            set(obj.h8, 'ZData',[obj.joint_locations(10, 3); obj.joint_locations(12, 3)]);

            set(obj.h9, 'XData', [obj.joint_locations(6, 1); obj.joint_locations(16, 1)]);
            set(obj.h9, 'YData',[obj.joint_locations(6, 2); obj.joint_locations(16, 2)]);
            set(obj.h9, 'ZData',[obj.joint_locations(6, 3); obj.joint_locations(16, 3)]);

            set(obj.h10, 'XData', [obj.joint_locations(9, 1); obj.joint_locations(11, 1)]);
            set(obj.h10, 'YData',[obj.joint_locations(9, 2); obj.joint_locations(11, 2)]);
            set(obj.h10, 'ZData',[obj.joint_locations(9, 3); obj.joint_locations(11, 3)]);

            set(obj.h11, 'XData', [obj.joint_locations(11, 1); obj.joint_locations(13, 1)]);
            set(obj.h11, 'YData',[obj.joint_locations(11, 2); obj.joint_locations(13, 2)]);
            set(obj.h11, 'ZData',[obj.joint_locations(11, 3); obj.joint_locations(13, 3)]);
            [x12, y12, z12] = obj.circle3D([obj.joint_locations(14, 1), obj.joint_locations(14, 2),obj.joint_locations(14, 3)],[1 0 0], obj.drive_wheel_rad);
            set(obj.h12,'XData', x12);
            set(obj.h12, 'YData', y12);
            set(obj.h12, 'ZData', z12);
            [x13, y13, z13] = obj.circle3D([obj.joint_locations(15, 1), obj.joint_locations(15, 2),obj.joint_locations(15, 3)],[1 0 0], obj.drive_wheel_rad);
            set(obj.h13,'XData', x13);
            set(obj.h13, 'YData', y13);
            set(obj.h13, 'ZData', z13);
            [x14, y14, z14] = obj.circle3D([obj.joint_locations(4, 1), obj.joint_locations(4, 2),obj.joint_locations(4, 3)],[1 0 0], obj.stab_wheel_rad);
            set(obj.h14,'XData', x14);
            set(obj.h14, 'YData', y14);
            set(obj.h14, 'ZData', z14);
            [x15, y15, z15] = obj.circle3D([obj.joint_locations(17, 1), obj.joint_locations(17, 2),obj.joint_locations(17, 3)],[1 0 0], obj.lhm_wheel_rad);
            set(obj.h15,'XData', x15);
            set(obj.h15, 'YData', y15);
            set(obj.h15, 'ZData', z15);
            [x16, y16, z16] = obj.circle3D([obj.joint_locations(18, 1), obj.joint_locations(18, 2),obj.joint_locations(18, 3)],[1 0 0], obj.lhm_wheel_rad);
            set(obj.h16,'XData', x16);
            set(obj.h16, 'YData', y16);
            set(obj.h16, 'ZData', z16);
            set(obj.h17,'XData', obj.com.location(1));
            set(obj.h17,'YData', obj.com.location(2));
            set(obj.h17,'ZData', obj.com.location(3));
            set(obj.h18,'XData', obj.com.location(1));
            set(obj.h18,'YData', obj.com.location(2));
            for i = 1:1:15
	            set(obj.h_com(i), 'XData', obj.com_locations(i,1));
	            set(obj.h_com(i), 'YData', obj.com_locations(i,2));
	            set(obj.h_com(i), 'ZData', obj.com_locations(i,3));
            end
    		drawnow;

		end

	end

end

% set(obj.h18, 'XData', obj.com_locations(1,1));
% set(obj.h18, 'YData', obj.com_locations(1,2));
% set(obj.h18, 'ZData', obj.com_locations(1,3));
% set(obj.h19, 'XData', obj.com_locations(2,1));
% set(obj.h19, 'YData', obj.com_locations(2,2));
% set(obj.h19, 'ZData', obj.com_locations(2,3));
