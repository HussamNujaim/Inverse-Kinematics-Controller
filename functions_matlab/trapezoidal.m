function [y,dy,ddy] = trapezoidal(y_i,y_f,dy_c,tf,t)
%
% trapezoidal velocity profile at instant t
% 
%   [y,dy,ddy] = trapezoidal(y_i,y_f,dy_c,t_f,t) 
%
%   input:
%       y_i     dim nx1     initial value
%       y_f     dim nx1     final value
%       dy_c    dim nx1     cruise velocity
%       tf      dim 1       final time
%       t       dim 1       output time
%   output:
%       y       dim nx1     position at t
%       dy      dim nx1     velocity at t
%       ddy     dim nx1     acceleration at t
%
% Gianluca Antonelli - Introduction to robotics/Sistemi robotici, 2020/2021

% input as columns
[n,m] = size(y_i);
if m>n
    y_i = y_i';
    y_f = y_f';
    dy_c = dy_c';
end
n = size(y_i,1);
y   = zeros(n,1);
dy  = zeros(n,1);
ddy = zeros(n,1);


% repeat for each of the joint
for i=1:n
    delta = y_f(i) - y_i(i);
    dy_c(i) = sign(delta)*abs(dy_c(i));
    if (delta==0)
        y(i)   = y_i(i);
        dy(i)  = 0;
        ddy(i) = 0;
    else
        % constraint verification for joint i
        dy_r = abs(delta/tf);
        err = (abs(dy_c(i)) <= dy_r)|(abs(dy_c(i)) > 2*dy_r);
        if (err==1)
            dy_c(i) = (2*dy_r);
            disp(dy_c(i))
            error('error in cruise velocity');
        else
            % evaluates t_c
            t_c = tf - delta/dy_c(i);
            % evaluates ddy_c
            ddy_c = dy_c(i)/t_c;
            % if on the time slots
            if (t<=t_c)
                y(i)   = y_i(i) + 0.5*ddy_c*t^2;
                dy(i)  = ddy_c*t;
                ddy(i) = ddy_c;
            elseif (t<=(tf-t_c))
                y(i)   = y_i(i) + dy_c(i)*(t-0.5*t_c);
                dy(i)  = dy_c(i);
                ddy(i) = 0;
            elseif (t<=tf)
                y(i)   = y_f(i) - 0.5*ddy_c*(t-tf)^2 ;
                dy(i)  = ddy_c*(tf-t);
                ddy(i) = -ddy_c;
            else
                y(i)   = y_f(i);
                dy(i)  = 0;
                ddy(i) = 0;
            end
        end
    end
end
