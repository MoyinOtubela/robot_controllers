
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

         hip_limit = pi/2;

    end

    methods(Access = protected, Hidden = true)

    	function [x, y, z] = FindCOMLocation(obj)
            x_m = (obj.com_locations(:, 1).*obj.mass);
            y_m = (obj.com_locations(:, 2).*obj.mass);
            z_m = (obj.com_locations(:, 3).*obj.mass);
            
            obj.com.location = [sum(x_m)/obj.total_mass;
                                sum(y_m)/obj.total_mass;
                                sum(z_m)/obj.total_mass];
    		% x_m = 0;        
    		% y_m = 0;
    		% z_m = 0; 
    		% len = size(obj.com_locations);
            % disp(len);
    		% for i = 1:len(1)
    		% 	x_m = x_m + obj.com_locations(i,1)*obj.mass(i);
    		% 	y_m = y_m + obj.com_locations(i,2)*obj.mass(i);
    		% 	z_m = z_m + obj.com_locations(i,3)*obj.mass(i);
    		% end

            % x_m = sum(x_m);
            % y_m = sum(y_m);
            % z_m = sum(z_m);


    		% obj.com.location = [x_m/obj.total_mass;
      %                           y_m/obj.total_mass;
      %                           z_m/obj.total_mass];
    	end

	end

    methods(Access = public)

    	function obj = LookupTableGenerator(obj)
			% positive stab = contract
            obj.x0 = [0 0 0.8 -0.2 0 0 0 0 0];
			obj.lb = [-pi/4 0 -1 0 -1 -1 0 0 0.6415];
			obj.ub = [0 1 0.8 0 0.383972435 0.383972435 0 0 0.6415];
			configure(obj, obj.x0);
			animate(obj);
    	end

    	function obj = set_head_mass(obj, mass)
    		obj.mass(15) = mass;
    	end

    	function [ssm] = findSSM(obj)
    		
    	end

    	function [ssm_delta] = findSSMDelta(obj)
    		x_o = (obj.joint_locations(1, 1)+(obj.joint_locations(4, 1)))/2;
    		y_o = (obj.joint_locations(1, 2)+(obj.joint_locations(4, 2)))/2;
    		% ssm_delta = sqrt((obj.com.location(1) - x_o)^2 + (obj.com.location(2) - y_o)^2 + (obj.com.location(3) - obj.desired_height)^2 );
            ssm_delta = sqrt(obj.K1*(obj.com.location(1) - x_o)^2 + obj.K2*(obj.com.location(2) - y_o)^2 + obj.K3*(obj.com.location(3) - obj.desired_height)^2 + (obj.hip_monitor - obj.hip_limit)^2 );
            % ssm_delta = sqrt(obj.K1*(obj.com.location(1) - x_o)^2 + obj.K2*(obj.com.location(2) - y_o)^2 + obj.K3*(obj.com.location(3) - obj.desired_height)^2);
            % ssm_delta = ssm_delta + abs(obj.hip_monitor - pi/2.5);

    		% ssm_delta = 10*abs(obj.com.location(1) - x_o) + 10*abs(obj.com.location(2) - y_o) + abs(obj.com.location(3) - obj.desired_height);
    		% ssm_delta =  (obj.joint_locations(4,3) - obj.stab_wheel_rad);
    		% ssm_delta = 10*sqrt((obj.com.location(1) - x_o)^2 + (obj.com.location(2) - y_o)^2) + abs(obj.com.location(3) - obj.desired_height);
    	end



    	function theta = run(obj, problem, x0, h)
			obj.desired_height = h;
			problem.x0 = x0;
			theta = fmincon(problem);

            % obj.refresh;
            
    	end

    	function result = solve(obj, joints)
    		obj.configure(joints);
			result = obj.findSSMDelta;
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

