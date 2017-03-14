% custom to the v2 robot
% assumes limb lengths and angles defined already 
% TO DO 
% make DH parameters members 
% update DH parameters by reading a configuration text file containing limb data etc. 

classdef aerobot< config_robot
    properties
        % LIMB MASSES (kg)
        joint_locations;
        origin = [0 0 0 1]';

        height;
        shank_height = 0;
        lhm_height;
        shank_rotation = 0;
    end

    methods(Abstract, Access = protected)
        obj = FindCOMLocation(obj);
    end
    
    methods(Hidden = true)

%       initialisation function 
        function  obj = aerobot
            title('Robbie Kinematics','FontSize',12);
            fig = figure(1);
            axis([-1 1 -1 1 -0.25 1.3]);
             hold on;
            xlabel('X-axis');ylabel('Y-axis');zlabel('Z-axis');
            hold on;
            grid on
            az = 90;
            el = 0;
            view(az, el);
        end

        function A = DH(obj, a, alpha, d, theta)
            A = [  cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha)  a*cos(theta)
                sin(theta) cos(theta)*cos(alpha)  -cos(theta)*sin(alpha) a*sin(theta)
                0           sin(alpha)              cos(alpha)              d
                0           0                        0                        1           ];
        end        
        

        function obj = update(obj)
            % shank footprint
            obj.a_0 = 0;                obj.alpha_0 = 0 + obj.shank_rotation;
            obj.d_0 = 0;    obj.theta_0 = 0;

            % shank_link
            obj.a_1 = 0;                obj.alpha_1 = 0;
            obj.d_1 = obj.drive_wheel_rad + obj.shank_height;    obj.theta_1 = 0;

            % shank_rotate 
            obj.a_2 = 0;                    obj.alpha_2 = 1.22173048 + obj.shank_angle;
            obj.d_2 = 0;     obj.theta_2 = 0;

            % obj.shank_len - obj.stab_knee_offset

            % shank_extend -> stab_link
            obj.a_3 = 0;     obj.alpha_3 = 0;
            obj.d_3 = 0.24;     obj.theta_3 = 0;

            % stab_link rotate
            obj.a_4 = 0;         obj.alpha_4 = 0.698131701 + obj.stab_angle;
            obj.d_4 = 0;                obj.theta_4 = 0;

            % extend stab_link
            obj.a_5 = 0;        obj.alpha_5 = 0;
            obj.d_5 = obj.stab_len;                obj.theta_5 = 0;

            % extend from shank_link to knee joint
            obj.a_6 = 0;        obj.alpha_6 = 0;
            obj.d_6 = 0.347;                obj.theta_6 = 0;

            % UPPER BODY
            % rotate thigh joint
            obj.a_7 = 0;        obj.alpha_7 = -2.44346095 + obj.knee_angle;
            obj.d_7 = 0;                obj.theta_7 = 0;

            % extend thigh joint
            obj.a_8 = 0;                obj.alpha_8 = 0;
            obj.d_8 = obj.thigh_len;    obj.theta_8 = 0;

            % rotate torso joint
            obj.a_9 = 0;                obj.alpha_9 = 1.57 + obj.hip_angle;
            obj.d_9 = 0; obj.theta_9 = 0;

            % extend torso joint to camera
            obj.a_10 = 0;               obj.alpha_10 = 0;   % by design 
            obj.d_10 = 0.42;               obj.theta_10 = 0;
            % obj.d_10 = 0.384;               obj.theta_10 = 0;
             % extend to right shoulder joint 
            obj.a_11 = -0.128;                obj.alpha_11 = obj.r_shoulder_angle;
            obj.d_11 = 0.3;    obj.theta_11 = 0;

             % extend to left shoulder joint 
            obj.a_12 = 0.128;                    obj.alpha_12 = obj.l_shoulder_angle;
            obj.d_12 = 0.3;     obj.theta_12 = 0;


            % left shoulder joint -> left arm
            obj.a_13 = -0.16;     obj.alpha_13 = 0;
            obj.d_13 = -0.265;     obj.theta_13 = 0;

            % right shoulder joint -> right arm
            obj.a_14 = 0.16;         obj.alpha_14 = 0;
            obj.d_14 = -0.265;                obj.theta_14 = 0;

            % left elbow
            obj.a_15 = 0;        obj.alpha_15 = 0;
            obj.d_15 = obj.left_arm_ext - 0.06;                obj.theta_15 = 0;

            % right elbow
            obj.a_16 = 0;        obj.alpha_16 = 0;
            obj.d_16 = obj.right_arm_ext - 0.06;                obj.theta_16 = 0;

            % WHEELS
            obj.a_17 = 0.1175;        obj.alpha_17 = 0;
            obj.d_17 = 0;                obj.theta_17 = 0;

            % right elbow
            obj.a_18 = -0.1175;        obj.alpha_18 = 0;
            obj.d_18 = 0;                obj.theta_18 = 0;

            % LHM torso joint
            obj.a_19 = 0;               obj.alpha_19 = 0;
            obj.d_19 = 0.03;  obj.theta_19 = 0;

            % obj.a_20 = 0;               obj.alpha_20 = -1.5708;
            % obj.d_20 = 0;  obj.theta_20 = 0;

            obj.a_20 = 0;               obj.alpha_20 = pi;
            obj.d_20 = 0;  obj.theta_20 = 0;

            % obj.a_21 = 0;               obj.alpha_21 = 0;
            % obj.d_21 = 0.05 + obj.lhm_position;  obj.theta_21 = 0;

            obj.a_21 = 0;               obj.alpha_21 = 0;
            obj.d_21 = 0.035 + obj.lhm_position;  obj.theta_21 = 0;


            obj.a_22 = -0.2125;               obj.alpha_22 = 0;
            obj.d_22 = 0;  obj.theta_22 = 0;

            obj.a_23 = 0.2125;               obj.alpha_23 = 0;
            obj.d_23 = 0;  obj.theta_23 = 0;

            obj.setupDH;
            obj.updateJoints;
        end

        function obj = updateJoints(obj)
        % FIND JOINT CO0RDINATES 
            % lower body
            shank_footprint = obj.A0;
            shank_link = obj.A0*obj.A1;
            stab_joint = shank_link*obj.A2*obj.A3;
            stab_wheel = stab_joint*obj.A4*obj.A5;
            knee_joint = shank_link*obj.A2*obj.A6;
            torso_link = knee_joint*obj.A7*obj.A8;
            camera_joint = torso_link*obj.A9*obj.A10;
            left_shoulder_link = torso_link*obj.A9*obj.A11;
            right_shoulder_link = torso_link*obj.A9*obj.A12;
            arm_left_link = left_shoulder_link*obj.A13;
            arm_right_link = right_shoulder_link*obj.A14;
            elbow_left_link = arm_left_link*obj.A15;
            elbow_right_link = arm_right_link*obj.A16;

            % obj.hip_monitor = acos(torso_link(3, 3));
            % sin_b = asin(torso_link(2, 3));
            % disp(torso_link)
            shank_left_wheel = shank_link*obj.A17;
            shank_right_wheel = shank_link*obj.A18;

            % lhm_torso_link = torso_link*obj.A9*obj.A19*obj.A20*obj.A21;
            % lhm_torso_link = torso_link*obj.A9*obj.A20*obj.A21;
            lhm_torso_link = torso_link*obj.A9*obj.A20*obj.A19*obj.A21;

            lhm_left_wheel = lhm_torso_link*obj.A22;
            lhm_right_wheel = lhm_torso_link*obj.A23;


            z = camera_joint(3,4) - torso_link(3,4);
            y = camera_joint(2,4) - torso_link(2,4);

            obj.hip_monitor = pi - atan2(z,y);

            % com_shank_footprint = shank_footprint*obj.DH(0.163037,0,0.159341,0)*obj.origin;
            com_shank_footprint = shank_link*obj.A2*obj.DH(0,0,0.1735,0)*obj.origin;
            % com_shank_footprint = shank_link*obj.A2*obj.DH(0,0,0.159341,0)*obj.origin;
            com_stab_link = stab_joint*obj.A4*obj.DH(0,0,0.04625,0)*obj.origin;
            com_thigh_link = knee_joint*obj.A7*obj.DH(0,0,0.1815,0)*obj.origin;
            com_torso_link = torso_link*obj.A9*obj.DH(0,0,0.192,0)*obj.origin;
            com_lhm_link = lhm_torso_link*obj.DH(0,0,-0.02,0)*obj.origin;
            com_shoulder_left_link = left_shoulder_link*obj.DH(-0.16,0,-0.15,0)*obj.origin;
            com_shoulder_right_link = right_shoulder_link*obj.DH(0.16,0,-0.15,0)*obj.origin;
            com_arm_left_link = arm_left_link*obj.DH(0,0,-0.1325,0)*obj.origin;
            com_arm_right_link = arm_right_link*obj.DH(0,0,-0.1325,0)*obj.origin;


            obj.joint_locations = [(shank_footprint*obj.origin)';
                                    (shank_link*obj.origin)';
                                    (stab_joint*obj.origin)';
                                    (stab_wheel*obj.origin)';
                                    (knee_joint*obj.origin)';
                                    (torso_link*obj.origin)';
                                    (camera_joint*obj.origin)';
                                    (left_shoulder_link*obj.origin)';
                                    (right_shoulder_link*obj.origin)';
                                    (arm_left_link*obj.origin)';
                                    (arm_right_link*obj.origin)';
                                    (elbow_left_link*obj.origin)';
                                    (elbow_right_link*obj.origin)';
                                    (shank_left_wheel*obj.origin)';
                                    (shank_right_wheel*obj.origin)';
                                    (lhm_torso_link*obj.origin)';
                                    (lhm_left_wheel*obj.origin)';
                                    (lhm_right_wheel*obj.origin)'];

            obj.com_locations = [com_shank_footprint';
                                 com_stab_link';
                                 com_thigh_link';
                                 com_torso_link';
                                 com_lhm_link';
                                 com_shoulder_left_link';
                                 com_shoulder_right_link';
                                 com_arm_left_link';
                                 com_arm_right_link';
                                 obj.joint_locations(4,:);
                                 obj.joint_locations(14,:);
                                 obj.joint_locations(15,:);
                                 obj.joint_locations(17,:);
                                 obj.joint_locations(18,:);
                                (camera_joint*obj.origin)';
                                ];
            
            obj.height = camera_joint(3, 4);
                            
            obj.FindCOMLocation;


        end
      
        function obj = animate(obj)
            % plot coordinates

            obj.h1 = plot3(obj.joint_locations(2:18, 1), obj.joint_locations(2:18, 2), obj.joint_locations(2:18, 3),'bo');
           
            % plot lines
            obj.h2 = plot3(obj.joint_locations(2:4, 1), obj.joint_locations(2:4, 2), obj.joint_locations(2:4, 3),'r', 'LineWidth', 3);
            obj.h3 = plot3([obj.joint_locations(3, 1); obj.joint_locations(5, 1)], [obj.joint_locations(3, 2); obj.joint_locations(5, 2)], [obj.joint_locations(3, 3); obj.joint_locations(5, 3)],'r', 'LineWidth', 3);
            obj.h4 = plot3(obj.joint_locations(5:7, 1), obj.joint_locations(5:7, 2), obj.joint_locations(5:7, 3),'r', 'LineWidth', 3);
           
            obj.h5 = plot3([obj.joint_locations(6, 1); obj.joint_locations(8, 1)], [obj.joint_locations(6, 2); obj.joint_locations(8, 2)], [obj.joint_locations(6, 3); obj.joint_locations(8, 3)],'r', 'LineWidth', 3);
            obj.h6 = plot3([obj.joint_locations(6, 1); obj.joint_locations(9, 1)], [obj.joint_locations(6, 2); obj.joint_locations(9, 2)], [obj.joint_locations(6, 3); obj.joint_locations(9, 3)],'r', 'LineWidth', 3);
            
            obj.h7 = plot3([obj.joint_locations(8, 1); obj.joint_locations(10, 1)], [obj.joint_locations(8, 2); obj.joint_locations(10, 2)], [obj.joint_locations(8, 3); obj.joint_locations(10, 3)],'r', 'LineWidth', 3);
            obj.h8 = plot3([obj.joint_locations(10, 1); obj.joint_locations(12, 1)], [obj.joint_locations(10, 2); obj.joint_locations(12, 2)], [obj.joint_locations(10, 3); obj.joint_locations(12, 3)],'b', 'LineWidth', 3);

            obj.h9 = plot3([obj.joint_locations(6, 1); obj.joint_locations(16, 1)], [obj.joint_locations(6, 2); obj.joint_locations(16, 2)], [obj.joint_locations(6, 3); obj.joint_locations(16, 3)],'b', 'LineWidth', 3);


            obj.h10 = plot3([obj.joint_locations(9, 1); obj.joint_locations(11, 1)], [obj.joint_locations(9, 2); obj.joint_locations(11, 2)], [obj.joint_locations(9, 3); obj.joint_locations(11, 3)],'r', 'LineWidth', 3);
            obj.h11 = plot3([obj.joint_locations(11, 1); obj.joint_locations(13, 1)], [obj.joint_locations(11, 2); obj.joint_locations(13, 2)], [obj.joint_locations(11, 3); obj.joint_locations(13, 3)],'b', 'LineWidth', 3);
            % plot wheelsb
            obj.h12 = plotCircle3D([obj.joint_locations(14, 1), obj.joint_locations(14, 2),obj.joint_locations(14, 3)],[1 0 0], obj.drive_wheel_rad,'g');
            obj.h13 = plotCircle3D([obj.joint_locations(15, 1), obj.joint_locations(15, 2),obj.joint_locations(15, 3)],[1 0 0], obj.drive_wheel_rad,'g');
            obj.h14 = plotCircle3D([obj.joint_locations(4, 1), obj.joint_locations(4, 2),obj.joint_locations(4, 3)],[1 0 0], obj.stab_wheel_rad,'g');

            obj.h15 = plotCircle3D([obj.joint_locations(17, 1), obj.joint_locations(17, 2),obj.joint_locations(18, 3)],[1 0 0], obj.lhm_wheel_rad,'g');
            obj.h16 = plotCircle3D([obj.joint_locations(18, 1), obj.joint_locations(18, 2),obj.joint_locations(18, 3)],[1 0 0], obj.lhm_wheel_rad,'g');

            obj.h17 = plot3(obj.com.location(1), obj.com.location(2), obj.com.location(3),'.k', 'MarkerSize', 60);
            
            obj.h18 = plot3(obj.com.location(1), obj.com.location(2), 0,'xk', 'MarkerSize', 20);

            % hold off;

            obj.h_com = [];
            for i = 1:15
                obj.h_com = [obj.h_com; plot3(obj.com_locations(i, 1), obj.com_locations(i, 2), obj.com_locations(i, 3), '.b', 'MarkerSize', 39)];
            end

        end      
        
    end
end
        