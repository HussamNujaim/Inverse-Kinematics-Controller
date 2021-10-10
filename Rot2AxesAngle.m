function a = Rot2AxesAngle(R)
%
% From rotation matrix to Axes-Angle
%
% function  a = Rot2AxesAngle(R)
%
% input:
%       R		dim 3x3     rotation matrix
%
% output:
%       a       dim 4x1      Axes-Angle
%


theta = acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
r= (1/(2*sin(theta))) * [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];

a = [r; theta];