% custom to the v2 robot
% assumes limb lengths and angles defined already 
% TO DO 
% make DH parameters members 
% update DH parameters by reading a configuration text file containing limb data etc. 

classdef v2_robot<handle
    properties
        % LIMB MASSES (kg)
        shank_mass; stab_mass; thigh_mass; arm_mass; torso_mass; LHM_mass;
        % WHEELS
        drive_wheel_rad; obj; stab_wheel_rad ;  lhm_wheel_rad ;
      
        % DH PARAMS
        % LOWER BODY
        % origin-r_wheel 
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
        a_19;   alpha_19;   d_19;   theta_19;  
%         a_20;   alpha_20;   d_20;   theta_20;   
        
        A1; A2; A3; A4; A5; A6; A7; A8; A9; A10; A11; A12; A13;
        A14; A15; A16; A17; A18; A19; A20 
        
        % COORDINATES
        origin = [0 0 0 1]';
        joint_locations;
    end
    
    methods
%       initialisation function 
        function  obj = robot(arg)
            robot_config_2;         % import joint params
            initial_config;         % setup DH matrices
            obj.setupDH();      
            obj.updateJoints();
        end

        function A = DH(obj, a, alpha, d, theta)
            A = [  cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha)  a*cosd(theta)
                sind(theta) cosd(theta)*cosd(alpha)  -cosd(theta)*sind(alpha) a*sind(theta)
                0           sind(alpha)              cosd(alpha)              d
                0           0                        0                        1           ];
%             c% = arg
        end        
        
        function obj = setupDH(obj)
          % SETUP DH params - cant call 
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
        end
        
        function obj = updateJoints(obj)
        % FIND JOINT CO-0RDINATES 
            % lower body
            wheel_rhs = obj.A1*obj.origin;
            wheel_lhs = obj.A2*obj.origin;
            stabiliser = obj.A3*obj.origin;
            stabiliser_wheel = obj.A3*obj.A4*obj.origin;

            % upper body  
            knee = obj.A5*obj.origin;
            hip = obj.A5*obj.A6*obj.origin;
            neck = obj.A5*obj.A6*obj.A7*obj.origin;
            shoulder_rhs = obj.A5*obj.A6*obj.A7*obj.A8*obj.origin;
            shoulder_lhs = obj.A5*obj.A6*obj.A7*obj.A9*obj.origin;
            elbow_rhs = obj.A5*obj.A6*obj.A7*obj.A8*obj.A10*obj.A11*obj.origin;
            elbow_lhs = obj.A5*obj.A6*obj.A7*obj.A9*obj.A12*obj.A13*obj.origin;

            % lower hip mechanism
            lhm = obj.A5*obj.A6*obj.A14*obj.A15*obj.origin;
            lhm_rwheel = obj.A5*obj.A6*obj.A14*obj.A15*obj.A16*obj.A17*obj.A18*obj.origin;
            lhm_lwheel = obj.A5*obj.A6*obj.A14*obj.A15*obj.A16*obj.A17*obj.A19*obj.origin;

            % return coordinates
            obj.joint_locations = [ obj.origin';
                                    wheel_rhs';
                                    wheel_lhs'; 
                                    stabiliser';
                                    stabiliser_wheel'; 
                                    knee'; 
                                    hip'; 
                                    neck'; 
                                    shoulder_rhs'; 
                                    shoulder_lhs';
                                    elbow_rhs'; 
                                    elbow_lhs'; 
                                    lhm'; 
                                    lhm_rwheel'; 
                                    lhm_lwheel'];            
        end
        
        
        
        
        function res  = IK_Update(obj, dt_shank, dt_knee, dt_hip )

            obj.theta_5 = obj.theta_5 + rad2deg(dt_shank);  % shank angle
            obj.theta_6 = obj.theta_6 + rad2deg(dt_knee);   % knee angle
            obj.theta_7 = obj.theta_7 + rad2deg(dt_hip);   % hip angle
            
%             % need to make thes public members
            obj.A5 = obj.DH(obj.a_5, obj.alpha_5, obj.d_5, obj.theta_5);
            obj.A6 = obj.DH(obj.a_6, obj.alpha_6, obj.d_6, obj.theta_6);
            obj.A7 = obj.DH(obj.a_7, obj.alpha_7, obj.d_7, obj.theta_7);
%                         
%             % get FK update 
            joint_A = (obj.A5*obj.origin);  % knee coordinates
            joint_B = (obj.A5*obj.A6*obj.origin); % hip coordinates
            joint_C = (obj.A5*obj.A6*obj.A7*obj.origin);  % neck coordinates
%                         
%             % get joint displacement vectors
            vect_CA = joint_C - joint_A;
            vect_CB = joint_C - joint_B;    
% 
            res = [joint_A, joint_B, joint_C, vect_CA, vect_CB];
          end
      
        function ff = ikine(obj, desired_ee)%    
          % calculate error in system 
            error = [ (desired_ee - obj.joint_locations(8,1:3)' ); 0; 0; 0];
            
          % get FK update 
            joint_A = obj.joint_locations(6,1:3)'  % knee coordinates
            joint_B = obj.joint_locations(7,1:3)'  % knee coordinates
            joint_C = obj.joint_locations(8,1:3)'  % neck coordinates
                                   
          % get joint displacement vectors
            vect_CA = joint_C - joint_A;
            vect_CB = joint_C - joint_B;             
            
            % save array
            joint_history = [obj.theta_5 obj.theta_6 obj.theta_7];
            
          % form Z-vectors to indicate angle of rotation for each joint
            z_1 = [0, 0, 1]';   % joint in Z-axis
            z_2 = [0, 0, 1]';   % joint in Z-axis
            z_3 = [0, 0, 1]';   % joint in Z-axis
% 
            tolerence = 0.01;   % set tolerence for convergence
            step = 0.01;        % weighting of angle change
            counter = 1;
            counter_max = input('How many iterations?');
          
            while(counter <=counter_max)   
                % calculate Jacobian and pseudo-inverse
                J = [   cross(z_1, joint_C(1:3))    cross(z_2, vect_CA(1:3))    cross(z_3, vect_CB(1:3));
                        z_1                         z_2                         z_3 ];
                pseudoJ = (inv(J'*J))*J';
                
                deltaTheta = (pseudoJ*error)*step;  % angles are in radians
                            
              
                % update the foward kinematics
                update = obj.IK_Update(deltaTheta(1) , deltaTheta(2), deltaTheta(3) );
                
                joint_A  = update(1:3,1);               
                joint_B  = update(1:3,2);                
                joint_C  = update(1:3,3);
                
                vect_CA = update(1:3,4);
                vect_CB = update(1:3,5);     
                
                angle_array = [obj.theta_5 obj.theta_6 obj.theta_7];
                joint_history = [joint_history(:,:); angle_array];
               
                error = [ (desired_ee - joint_C ); 0; 0; 0]
                err = sqrt(error(1)^2+error(2)^2);
% 
                counter = counter +1
                if (err<tolerence)
                    disp('TOLERANCE ACHIEVED');
                    disp(obj.theta_5);
                    disp(obj.theta_6);
                    disp(obj.theta_7);
                    break;
                end  
            end
            % update parameters to excel file
            filename = 'IK_xlsx.xlsx';
            sheet = 1;

            num_rows = counter;
            last_row = strcat('E',num2str(num_rows+1))
            data_range = strcat('C2:',last_row)
            xlswrite(filename,joint_history,sheet,data_range);

            % log shortened version
            list_limit = 200;
            testrange_after = [1:(counter/list_limit):counter];
            testrange_before = [1:1:counter];
            joint_history_shortened = interp1(testrange_before, joint_history, testrange_after);
            xlswrite(filename,joint_history_shortened,sheet,'G2:I202');
        end
        
        function obj = animate(obj)
        % setup figure
            title('Robbie Kinematics','FontSize',12);
            figure(1);
            axis([-2 2 -2 2 -0.25 3.5]); hold on;
            xlabel('X-axis');ylabel('Y-axis');zlabel('Z-axis');
            grid on
            az = 0;
            el = 0;
            view(az, el);
            
        % plot coordinates
            plot3(obj.joint_locations(1:15,2),obj.joint_locations(1:15,3),obj.joint_locations(1:15,1),'o');
            hold on;
        % PLOT LINES
        % wheelbase
            plot3(obj.joint_locations(2:3,2),obj.joint_locations(2:3,3),obj.joint_locations(2:3,1),'r','LineWidth',3);
        % draw drive_wheels
            plotCircle3D( [ obj.joint_locations(2,2), obj.joint_locations(2,3), obj.joint_locations(2,1) ] ,[0, 1, 0], obj.drive_wheel_rad, 'g')
            plotCircle3D( [ obj.joint_locations(3,2), obj.joint_locations(3,3), obj.joint_locations(3,1) ] ,[0, 1, 0], obj.drive_wheel_rad, 'g')
        % shank
            plot3( [obj.joint_locations(1,2);obj.joint_locations(6,2)],[obj.joint_locations(1,3); obj.joint_locations(6,3)],[obj.joint_locations(1,1); obj.joint_locations(6,1)],'r','LineWidth',3);
        % stabiliser
            plot3( [obj.joint_locations(4,2);obj.joint_locations(5,2)],[obj.joint_locations(4,3); obj.joint_locations(5,3)],[obj.joint_locations(4,1); obj.joint_locations(5,1)],'r','LineWidth',3);
            plotCircle3D( [ obj.joint_locations(5,2), obj.joint_locations(5,3), obj.joint_locations(5,1) ] ,[0, 1, 0], 0.25, 'r')% 
        % thigh
            plot3(obj.joint_locations(6:7,2),obj.joint_locations(6:7,3),obj.joint_locations(6:7,1),'r','LineWidth',3);
        % torso
            plot3(obj.joint_locations(7:8,2),obj.joint_locations(7:8,3),obj.joint_locations(7:8,1),'r','LineWidth',3);
        % shoulderwidth
            plot3(obj.joint_locations(9:10,2),obj.joint_locations(9:10,3),obj.joint_locations(9:10,1),'r','LineWidth',3);
        % right arm
            plot3( [obj.joint_locations(9,2);obj.joint_locations(11,2)],[obj.joint_locations(9,3); obj.joint_locations(11,3)],[obj.joint_locations(9,1); obj.joint_locations(11,1)],'r','LineWidth',3);
        % left arm
            plot3( [obj.joint_locations(10,2);obj.joint_locations(12,2)],[obj.joint_locations(10,3); obj.joint_locations(12,3)],[obj.joint_locations(10,1); obj.joint_locations(12,1)],'r','LineWidth',3);
        % hip-lhm
            plot3( [obj.joint_locations(7,2);obj.joint_locations(13:15,2)],[obj.joint_locations(7,3); obj.joint_locations(13:15,3)],[obj.joint_locations(7,1); obj.joint_locations(13:15,1)],'b','LineWidth',3);
            plotCircle3D( [ obj.joint_locations(14,2), obj.joint_locations(14,3), obj.joint_locations(14,1) ] ,[0, 1, 0], obj.lhm_wheel_rad, 'g')
            plotCircle3D( [ obj.joint_locations(15,2), obj.joint_locations(15,3), obj.joint_locations(15,1) ] ,[0, 1, 0], obj.lhm_wheel_rad, 'g')
            hold on;            
        end      
        
    end
end
        
           






