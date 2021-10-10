function J = Jacobian(DH)
%
% Computes the geometric Jacobian (only rotational joints)
%
% function J = Jacobian(DH)
%
% input:
%       DH     dim nx4     Denavit-Hartenberg table
%
% output:
%       J      dim 6xn     Geometric Jacobian
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2020/2021

% p: 3xn matrix, the generic column is the position of frame i expressed in inertial frame
% z: 3xn matrix, the generic column is the z-versor of frame i expressed in inertial frame

n = size(DH,1);
J = zeros(6,n);
p = zeros(3,n);
z = zeros(3,n);

% compute homog. transf. from base frame
T0 = zeros(4,4,n);
for i=1:n
    T_i = Homogeneous(DH(i,:));
    if i==1
        T0(:,:,i) = T_i;
    else
        T0(:,:,i) = T0(:,:,i-1)*T_i;
    end
    p(:,i) = T0(1:3,4,i);
    z(:,i) = T0(1:3,3,i);
end

z0 = [0 0 1]';
p0 = [0 0 0]';
J(:,1) = [cross(z0,p(:,n)-p0);
            z0];
for i=2:n
    J(:,i) = [cross(z(:,i-1),p(:,n)-p(:,i-1));
            z(:,i-1)];
end





