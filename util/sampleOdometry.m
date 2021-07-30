function [xk, u] = sampleOdometry(o_1,o,x,alpha)
%SAMPLEODOMETRY Summary: Transform two poses in a comand
%   Detailed explanation goes here
% o_1 input parameter is the odometry pose in t-1 is a vector like [ x  y thetha ]  
% o input parameter is the odometry in time t is a vector like [ x  y thetha ]
% x is the state vector 

x_bar_p = o(1);
y_bar_p = o(2);
theta_bar_p = o(3);

x_bar = o_1(1);
y_bar = o_1(2);
theta_bar = o_1(3);

%% c an a just for debugging DETETE IT
%%c = normalizeAngle(3.14+1.57);
a = sqrt(power(x_bar - x_bar_p, 2) + power(y_bar - y_bar_p, 2));



if  a < .01  
    d_rot1 = 0; % Si solo gira  y este valor no es cero entonces  d_rot2 = - d_rot1 y el angulo final es practicamente el mismo  que el inicial :o alv
else
    d_rot1 = normalizeAngle(normalizeAngle(atan2(y_bar_p - y_bar, x_bar_p - x_bar )) - normalizeAngle(theta_bar)); %atan2(y_bar_p - y_bar, x_bar_p - x_bar ) - theta_bar;

d_trans1 = sqrt(  power(x_bar - x_bar_p, 2)  +  power(y_bar - y_bar_p, 2)  );
d_rot2 = normalizeAngle(normalizeAngle(theta_bar_p) - normalizeAngle(theta_bar + d_rot1)); %theta_bar_p - theta_bar - d_rot1;
    
d_rot1_hat =  d_rot1 -normrnd(0, alpha(1) * power(d_rot1,2) + alpha(2) * power(d_trans1,2) );
d_trans1_hat = d_trans1 -normrnd(0,  alpha(3) * power(d_trans1,2) + alpha(4) * power(d_rot1,2) + alpha(4) * power(d_rot2,2));
d_rot2_hat = d_rot2 -normrnd(0, alpha(1) * power(d_rot2,2) + alpha(2) * power(d_trans1,2) );
    
x_o(1) = x(1) + d_trans1_hat * cos( x(3) + d_rot1_hat );
x_o(2) = x(2) + d_trans1_hat * sin( x(3) + d_rot1_hat );


u = [  d_trans1_hat; d_rot1_hat+x(3) ];
%*theta_plus_rotation1 = x_vector(2) + d_rot1_hat;
%*translation = d_trans1_hat;

x_o(3) = x(3) + d_rot1_hat + d_rot2_hat;

xk = [x_o(1); x_o(2); x_o(3)];
  
    
end

