
classdef LookupTableGenerator < aerobot
    properties


         desired_height;

         com_locations;

         com = COM;

         x0;
         lb;
         ub;

         K1=1;
         K2=1;
         K3=1;
         K4=1;
         K5=1;

         hip_limit = pi/2;

         com_height_limit;

         obstacle_height = 0;


    end

    methods(Access = protected, Hidden = true)

    	function [x, y, z] = FindCOMLocation(obj)
            x_m = (obj.com_locations(:, 1).*obj.mass);
            y_m = (obj.com_locations(:, 2).*obj.mass);
            z_m = (obj.com_locations(:, 3).*obj.mass);
            
            obj.com.location = [sum(x_m)/obj.total_mass;
                                sum(y_m)/obj.total_mass;
                                sum(z_m)/obj.total_mass];

    	end

	end

    methods(Access = public)

    	function obj = LookupTableGenerator(obj)
			% positive stab = contract
            obj.x0 = [0 0 0.8 0.2 0 0 0 0];
            % obj.lb = [-pi/4 0 -1 0 -1 -1 0 0];
            obj.ub = [0 1 0.8 0 0.383972435 0.383972435 0 0];
            obj.lb = [-pi/4 0 -1 0 -pi -pi 0 0];
            % obj.ub = [0 1 0.8 0 pi pi 0 0];
			configure(obj, obj.x0);
			animate(obj);
    	end

    	function obj = set_head_mass(obj, mass)
    		obj.mass(15) = mass;
    	end

    	function [ssm] = findSSM(obj, stab, lhm, shank)

            shank_location = obj.joint_locations(2, :);
            stab_location = obj.joint_locations(4, :);
            lhm_location_lw = obj.joint_locations(17, :);
            lhm_location_rw = obj.joint_locations(18, :);
            % disp(shank_location)
            lhm_location = [];
            lhm_location(1) = (lhm_location_lw(1) + lhm_location_rw(1))/2;
            lhm_location(2) = (lhm_location_lw(2) + lhm_location_rw(2))/2;

            if(stab && lhm)
                d1 = sqrt(  ( lhm_location(1) - obj.com.location(1) )^2  + (lhm_location(2) - obj.com.location(2))^2 );
                d2 = sqrt(  ( stab_location(1) - obj.com.location(1) )^2  + (stab_location(2) - obj.com.location(2))^2 );
                if(d1<d2)
                    ssm = d1;
                else
                    ssm = d2;
                end
                return;
            elseif (stab && shank)
                d1 = sqrt(  ( shank_location(1) - obj.com.location(1) )^2  + (shank_location(2) - obj.com.location(2))^2 );
                d2 = sqrt(  ( stab_location(1) - obj.com.location(1) )^2  + (stab_location(2) - obj.com.location(2))^2 );
                if(d1<d2)
                    ssm = d1;
                else
                    ssm = d2;
                end
                return;
            elseif (shank && lhm)
                d1 = sqrt(  ( shank_location(1) - obj.com.location(1) )^2  + (shank_location(2) - obj.com.location(2))^2 );
                d2 = sqrt(  ( lhm_location(1) - obj.com.location(1) )^2  + (lhm_location(2) - obj.com.location(2))^2 );
                if(d1<d2)
                    ssm = d1;
                else
                    ssm = d2;
                end
                return;
            end
                    
            ssm = 1e-100;
    	end
        function [act] = convert_to_robot_output(obj, theta)
        %This function converts the angles generated from the solver into
        %actionable commands for the real robot
            act = zeros(8,1);
            k = (pi/2 -  (1.22173048 + theta(1)));
            h = obj.shank_to_stab*sin(k - obj.shank_rotation);
            act(1) = pi/2 - (1.22173048 + obj.shank_rotation + theta(1)) + asin((h+(obj.drive_wheel_rad-obj.stab_wheel_rad+obj.shank_height) - obj.stab_height)/(obj.stab_len)) - 0.698131701;
            act(2:8) = theta(2:8);
            act(4) = theta(4);

        end


        function contact = determine_contact(obj, support, wheel_radius)
            contact = ( abs(support - wheel_radius) <=  0.01);
        end


        function [ssm_delta] = findSSMDelta(obj)

            % stab = obj.determine_contact(obj.joint_locations(4, 3) - obj.stab_height, obj.stab_wheel_rad);
            % stab = obj.determine_contact(obj.joint_locations(4, 3), obj.stab_wheel_rad);
            stab = obj.determine_contact(obj.joint_locations(4, 3) - obj.obstacle_height, obj.stab_wheel_rad);
            shank = obj.determine_contact(obj.joint_locations(2, 3)-obj.obstacle_height, obj.drive_wheel_rad);
            lhm = obj.determine_contact(obj.joint_locations(16, 3), obj.lhm_wheel_rad);

            ssm = obj.findSSM(stab, shank, lhm);

            if(stab && lhm)
                x_o = (obj.joint_locations(4, 1)+(obj.joint_locations(18, 1)))/2;
                y_o = (obj.joint_locations(4, 2)+(obj.joint_locations(18, 2)))/2;
                a = (obj.com.location(1) - x_o)^2;
                b = (obj.com.location(2) - y_o)^2;
                c = (obj.com.location(3) - obj.desired_height)^2;
                d = (obj.hip_monitor - obj.hip_limit)^2;
                e =  1/ssm;
                ssm_delta = (obj.K1*(a) + obj.K2*(b) + obj.K3*(c) + obj.K4*(d) + obj.K5*e);
                return;
            elseif (stab && shank)
                x_o = (obj.joint_locations(4, 1)+(obj.joint_locations(2, 1)))/2;
                y_o = (obj.joint_locations(4, 2)+(obj.joint_locations(2, 2)))/2;
                a = (obj.com.location(1) - x_o)^2;
                b = (obj.com.location(2) - y_o)^2;
                c = (obj.com.location(3) - obj.desired_height)^2;
                d = (obj.hip_monitor - obj.hip_limit)^2;
                e = 1/ssm;
                ssm_delta = (obj.K1*(a) + obj.K2*(b) + obj.K3*(c) + obj.K4*(d) + obj.K5*e);
                return;
            elseif (shank && lhm)
                x_o = (obj.joint_locations(2, 1)+(obj.joint_locations(16, 1)))/2;
                y_o = (obj.joint_locations(2, 2)+(obj.joint_locations(16, 2)))/2;
                a = (obj.com.location(1) - x_o)^2;
                b = (obj.com.location(2) - y_o)^2;
                c = (obj.com.location(3) - obj.desired_height)^2;
                d = (obj.hip_monitor - obj.hip_limit)^2;
                e = 1/ssm;
                ssm_delta = (obj.K1*(a) + obj.K2*(b) + obj.K3*(c) + obj.K4*(d) + obj.K5*e);
                return;
            end
                    
            ssm_delta = 1e100;
            
        end

        function [ssm_delta] = findSSMDelta2(obj)

            % stab = obj.determine_contact(obj.joint_locations(4, 3) - obj.stab_height, obj.stab_wheel_rad);
            % stab = obj.determine_contact(obj.joint_locations(4, 3), obj.stab_wheel_rad);
            stab = obj.determine_contact(obj.joint_locations(4, 3) - obj.obstacle_height, obj.stab_wheel_rad);
            shank = obj.determine_contact(obj.joint_locations(2, 3)-obj.obstacle_height, obj.drive_wheel_rad);
            lhm = obj.determine_contact(obj.joint_locations(16, 3), obj.lhm_wheel_rad);

            ssm = obj.findSSM(stab, shank, lhm);

            x_o = (obj.joint_locations(2, 1)+(obj.joint_locations(16, 1)))/2;
            y_o = (obj.joint_locations(2, 2)+(obj.joint_locations(16, 2)))/2;
            a = (obj.com.location(1) - x_o)^2;
            b = (obj.com.location(2) - y_o)^2;
            c = (obj.com.location(3) - obj.desired_height)^2;
            d = (obj.hip_monitor - obj.hip_limit)^2;
            e = 1/ssm;
            ssm_delta = (obj.K1*(a) + obj.K2*(b) + obj.K3*(c) + obj.K4*(d) + obj.K5*e);
                % return;
            % end
                    
            % ssm_delta = 1e100;
            return;
            
        end

        function [ssm_delta] = findSSMDelta3(obj)

            % stab = obj.determine_contact(obj.joint_locations(4, 3) - obj.stab_height, obj.stab_wheel_rad);
            % stab = obj.determine_contact(obj.joint_locations(4, 3), obj.stab_wheel_rad);
            % stab = obj.determine_contact(obj.joint_locations(4, 3) - obj.obstacle_height, obj.stab_wheel_rad);
            % shank = obj.determine_contact(obj.joint_locations(2, 3)-obj.obstacle_height, obj.drive_wheel_rad);
            % lhm = obj.determine_contact(obj.joint_locations(16, 3), obj.lhm_wheel_rad);

            % ssm = obj.findSSM(stab, shank, lhm);

            x_o = (obj.joint_locations(4, 1)+(obj.joint_locations(2, 1)))/2;
            y_o = (obj.joint_locations(4, 2)+(obj.joint_locations(2, 2)))/2;
            a = (obj.com.location(1) - x_o)^2;
            b = (obj.com.location(2) - y_o)^2;
            c = (obj.com.location(3) - obj.desired_height)^2;
            d = (obj.hip_monitor - obj.hip_limit)^2;
            % e = 1/ssm;
            ssm_delta = (obj.K1*(a) + obj.K2*(b) + obj.K3*(c) + obj.K4*(d));
                % return;
            % end
                    
            % ssm_delta = 1e100;
            return;
            
        end



    	function theta = run(obj, problem, x0, h)
            obj.desired_height = h;
            problem.x0 = x0;
            theta = fmincon(problem)
            theta(4) = obj.lhm_position;
            % obj.refresh;
            
    	end

        function result = solve(obj, joints)
            obj.configure(joints);
            result = obj.findSSMDelta;
            % obj.refresh();  %uncomment for live update
            % fprintf('OBJ = %g\n',result)
        end

        function result = solve_stab_lhm(obj, joints)
            obj.configure(joints);
            result = obj.findSSMDelta2;
            % obj.refresh();  %uncomment for live update
            % fprintf('OBJ = %g\n',result)
        end
        function result = solve_stab_shank(obj, joints)
            obj.configure(joints);
            result = obj.findSSMDelta3;
            % obj.refresh();  %uncomment for live update
            % fprintf('OBJ = %g\n',result)
        end

    	function output = testSolve(obj, x, y)
    		output = cos(x + y);
    	end
    	function output = testSolve2(obj, x)
    		output = cos(x(1) + x(2));
    	end
	end
    
end

            % x_m = 0;        
            % y_m = 0;
            % z_m = 0; 
            % len = size(obj.com_locations);
            % disp(len);
            % for i = 1:len(1)
            %   x_m = x_m + obj.com_locations(i,1)*obj.mass(i);
            %   y_m = y_m + obj.com_locations(i,2)*obj.mass(i);
            %   z_m = z_m + obj.com_locations(i,3)*obj.mass(i);
            % end

            % x_m = sum(x_m);
            % y_m = sum(y_m);
            % z_m = sum(z_m);


            % obj.com.location = [x_m/obj.total_mass;
      %                           y_m/obj.total_mass;
      %                           z_m/obj.total_mass];
