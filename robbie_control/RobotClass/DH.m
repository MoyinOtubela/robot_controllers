function [A, Asym] = DH(a, alpha, d, theta)
    A = [  cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha)  a*cosd(theta)
            sind(theta) cosd(theta)*cosd(alpha)  -cosd(theta)*sind(alpha) a*sind(theta)
            0           sind(alpha)              cosd(alpha)              d
            0           0                        0                        1           ];

    if (a~=0)
        a = sym('a');
    end
    if (alpha~=0)
        alpha = sym('alpha');
    end
    if (d~=0)
        d = sym('d');
    end
    if (theta~=0)
        theta = sym('theta');
    end
    
    Asym = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha)  a*cos(theta)
            sin(theta) cos(theta)*cos(alpha)  -cos(theta)*sin(alpha) a*sin(theta)
            0           sin(alpha)              cos(alpha)              d
            0           0                        0                        1           ];
        
        
        

    
    