function R = AxesAngle2Rot(a)
%
% From Axes-Angle to rotation matrix 
%
% function R = AxesAngle2Rot(a)
%
% input:
%       a       dim 4x1      Axes-Angle    
%
% output:

%       R		dim 3x3     rotation matrix
%
a=a'
theta = a(4,1);
x = a(1,1); y =a(2,1); z=a(3,1); 

s = sin(theta);
c = cos(theta);

r11 = x^2 * (1-c) + c;
r12 = x*y*(1-c)-z*s;
r13 = x*z*(1-c)+y*s;

r21 = x*y*(1-c) - z*s;
r22 = y^2 *(1-c)+c;
r23 = y*z*(1-c)-x*s;

r31 = x*z *(1-c) - y*s;
r32 = y*z *(1-c) + x*s;
r33 = z^2 *(1-c)+c;

R=[r11 r12 r13
    r21 r22 r23
    r31 r32 r33];
