function [act] = convert_to_robot_output(theta)
%This function converts the angles generated from the solver into
%actionable commands for the real robot
    act = zeros(8,1);
    k = (pi/2 -  (1.22173048 + theta(1)));
	h = 0.24*sin(k);
    act(1) = pi/2 - (1.22173048 + theta(1)) + asin( (h+(0.0247))/(0.25)) - 0.698131701;
    act(2:3) = theta(2:3);
    act(4) = -theta(4);
    act(5:8) = theta(5:8);
end

