function  T0 = DirectKinematics(DH)
%
% Homogeneous transformation matrices with respect to the arm base frame 
% T0 = DirectKinematics(DH)
%
% input:
%       DH     dim nx4     Denavit-Hartenberg table
%
% output:
%       T      dim 4x4xn   Homogeneous transformation matrices with respect to the arm base frame    
%


    n = size(DH,1);
    T0 = zeros(4,4,n);
    T  = zeros(4,4,n);

    % Homogeneous transformation matrix between consecutive frames according to DH convention
    for i=1:n
        T(:,:,i) = Homogeneous(DH(i,:));
    end

    % Homogeneous transformation matrices with respect to the arm base frame. T0(:,:,n) contains the homogeneous 
    % transformation matrix between the end effector and the arm base frame
    T0(:,:,1) = T(:,:,1);

    for i=2:n
        T0(:,:,i) = T0(:,:,i-1) * T(:,:,i);
    end

end
