function [h] = plotCircle3D(center,normal,radius,color)
% center: should be a 3D vector in form [x, y, z]
% normal: should be a 3D vector in form [x, y, z]


% sets circle color to blue if no color is passed
if nargin < 4
   color = 'b'
end

theta=0:0.01:2*pi;


v=null(normal);
points=repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
x = points(1,:);
y = points(2,:);
z = points(3,:);
h = plot3(x, y, z,color);

end