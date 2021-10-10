function T = Homogeneous(DH_row)
%
% Homogeneous transformation matrix between consecutive frames according to DH convention
%
% T = Homogeneous_dh(DH_i)
%
% input:
%       DH_row     dim 1x4     row i of the Denavit-Hartenberg table
%
% output:
%       T      dim 4x4      Homogeneous transformation matrix
%


    a = DH_row(1);
    alpha = DH_row(2);
    d = DH_row(3);
    theta = DH_row(4);

    ct = cos(theta);
    st = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);

    T = [ct -st*ca st*sa a*ct;
         st ct*ca -ct*sa a*st;
         0   sa  ca  d;
         0   0   0   1];

end
