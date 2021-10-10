
%----------------------------------%
%                                  %
%   Mask JointPosition DH2Jaco     %
%                                  %
%----------------------------------%



function q_Jaco = mask_q_DH2Jaco(q)
   
  
    q_Jaco(1) = -q(1);
    q_Jaco(2) = q(2) + pi;
    q_Jaco(3) = q(3);
    q_Jaco(4) = q(4);
    q_Jaco(5) = q(5);
    q_Jaco(6) = q(6) + pi;
    q_Jaco(7) = -q(7);
    

end
