% syms alpha theta

% solx = solve( theta - pi/2  ==  pi - alpha - asin((0.24*sin(pi/2 - theta) + 0.05)/0.356 ), theta
syms shank_angle stab_angle 
% solx = solve([shank_angle == pi/2 - stab_angle + asin(((0.24*sin(pi/2 - shank_angle)) + (0.02389))/0.25)], [shank_angle, stab_angle], 'ReturnConditions', true)
solx = solve(0.24*sin(pi/2 - shank_angle) + 0.02389 == 0.25*sin(shank_angle + stab_angle - pi/2), stab_angle)

% syms z y
% solx2= solve(y == pi/2 - z - asin((24*sin(z - pi/2))/25 - 2389/25000), z, y, 'ReturnConditions', true)