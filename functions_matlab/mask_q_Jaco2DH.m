
%----------------------------------%
%                                  %
%   Mask JointPosition Jaco2DH     %
%                                  %
%----------------------------------%



function q_DH = mask_q_Jaco2DH(q)
    
    
    q_DH(1) = -q(1);
    q_DH(2) = q(2) - pi;
    q_DH(3) = q(3);
    q_DH(4) = q(4);
    q_DH(5) = q(5);
    q_DH(6) = q(6) - pi;
    q_DH(7) = -q(7);
    

end
